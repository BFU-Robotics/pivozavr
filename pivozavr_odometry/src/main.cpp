#include <chrono>
#include <functional>
#include <memory>

#include "retrans.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "pivozavr_interfaces/msg/wheel_info.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <fstream>
#include <iomanip>


#define TICK_TO_RAD 0.00253

using std::placeholders::_1;

class Odometry : public rclcpp::Node
{
  public:
    Odometry() : Node("pivozavr_odometry")
    {
        m_velocitySub = this->create_subscription<pivozavr_interfaces::msg::WheelInfo>(
            "wheel_velocities", 10, std::bind(&Odometry::velocityCallback, this, _1));
        m_joint_states_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        m_lastTime = this->now();

        m_wheelCmdPub = this->create_publisher<pivozavr_interfaces::msg::WheelInfo>("wheel_commands", 10);
        m_cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&Odometry::cmdVelCallback, this, _1));

        m_odomPub = this->create_publisher<nav_msgs::msg::Odometry>("odometry/unfiltered", 10);
    	  m_tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

  private:

    void velocityCallback(const pivozavr_interfaces::msg::WheelInfo & msg)
    {
        double velocityFL = msg.fl;
        double velocityFR = msg.fr;
        double velocityML = msg.ml;
        double velocityMR = msg.mr;
        double velocityRL = msg.rl;
        double velocityRR = msg.rr;

        // Формирование данных для joint state publisher
        auto joint_states_msg = std::make_unique<sensor_msgs::msg::JointState>();
        joint_states_msg->header.frame_id = "base_link";
        joint_states_msg->header.stamp = this->now();
        joint_states_msg->name.emplace_back("front_left_wheel_joint");
        joint_states_msg->name.emplace_back("front_right_wheel_joint");
        joint_states_msg->name.emplace_back("mid_left_wheel_joint");
        joint_states_msg->name.emplace_back("mid_right_wheel_joint");
        joint_states_msg->name.emplace_back("rear_left_wheel_joint");
        joint_states_msg->name.emplace_back("rear_right_wheel_joint");

        rclcpp::Duration duration(rclcpp::Duration::from_nanoseconds(
                this->now().nanoseconds() - m_lastTime.nanoseconds()));
        double dt = duration.seconds();
        m_lastTime = this->now();

        m_positionFL += velocityFL*dt;
        m_positionFR += velocityFR*dt;
        m_positionML += velocityML*dt;
        m_positionMR += velocityMR*dt;
        m_positionRL += velocityRL*dt;
        m_positionRR += velocityRR*dt;

        joint_states_msg->position.push_back(m_positionFL);
        joint_states_msg->position.push_back(m_positionFR);
        joint_states_msg->position.push_back(m_positionML);
        joint_states_msg->position.push_back(m_positionMR);
        joint_states_msg->position.push_back(m_positionRL);
        joint_states_msg->position.push_back(m_positionRR);

        m_joint_states_pub->publish(std::move(joint_states_msg));

        // Расчет одометрии
        float wL = (velocityFL + velocityML + velocityRL)/3;
        float wR = (velocityFR + velocityMR + velocityRR)/3;

        //std::cerr << wL << " " << wR << " " << wR-wL << std::endl;

        float vx = (wR + wL)*m_R/2;
        float wz = (wR - wL)*m_R/m_S;
        //std::cerr << vx << " " << wz << std::endl;

        m_rotation += wz*dt;
		    m_positionX += std::cos(m_rotation)*vx*dt;
		    m_positionY += std::sin(m_rotation)*vx*dt;

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";
        odom_msg.header.stamp = this->now();
        odom_msg.pose.pose.position.x = m_positionX;
        odom_msg.pose.pose.position.y = m_positionY;
        odom_msg.pose.pose.position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, m_rotation);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = 0;
        odom_msg.twist.twist.angular.z = wz;
        m_odomPub->publish(odom_msg);

        // Публикация TF
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_footprint";
        odom_tf.header.stamp = this->now();
        odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
        odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
        odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
        odom_tf.transform.rotation = odom_msg.pose.pose.orientation;
        //m_tfBroadcaster->sendTransform(odom_tf);
    }


    void cmdVelCallback(const geometry_msgs::msg::Twist & msg)
    {        
        /*
        RCLCPP_INFO(this->get_logger(), "%f", msg.linear.x);
        RCLCPP_INFO(this->get_logger(), "%f", msg.linear.y);
        RCLCPP_INFO(this->get_logger(), "%f", msg.angular.z);
        */

        float vx = msg.linear.x;
        float wz = msg.angular.z;

        // Расчет угловых скоростей левой и правой стороны
        float wL = vx/m_R - m_S*wz/(2*m_R);
        float wR = vx/m_R + m_S*wz/(2*m_R);

        // Подготовка и отправка сообщения
        auto wheelCmdMsg = pivozavr_interfaces::msg::WheelInfo();
        wheelCmdMsg.fl = wL;
        wheelCmdMsg.fr = wR;
        wheelCmdMsg.ml = wL;
        wheelCmdMsg.mr = wR;
        wheelCmdMsg.rl = wL;
        wheelCmdMsg.rr = wR;

        m_wheelCmdPub->publish(wheelCmdMsg);
    }

    const float m_R = 0.13;     // радиус в метрах
    const float m_S = 0.573;    // расстояние между колесами

    float m_positionFL = 0;
    float m_positionFR = 0;
    float m_positionML = 0;
    float m_positionMR = 0;
    float m_positionRL = 0;
    float m_positionRR = 0;

    float m_rotation = 0;
    float m_positionX = 0;
    float m_positionY = 0;

    rclcpp::Time m_lastTime;
    rclcpp::Subscription<pivozavr_interfaces::msg::WheelInfo>::SharedPtr m_velocitySub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_states_pub;
    rclcpp::Publisher<pivozavr_interfaces::msg::WheelInfo>::SharedPtr m_wheelCmdPub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelSub;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odomPub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
