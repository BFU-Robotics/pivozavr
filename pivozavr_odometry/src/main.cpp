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
        m_encodes_sub = this->create_subscription<pivozavr_interfaces::msg::WheelInfo>(
            "wheel_velocities", 10, std::bind(&Odometry::encodes_callback, this, _1));

        m_joint_states_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        m_last_time = this->now();

        /*odom_class = std::make_unique<OdometryClass>(0.04, 0.1215, 0.11);
        m_pub = this->create_publisher<telerobot_interfaces::msg::Motor>("wheel_commands", 10);
        m_odom_commands_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&Odometry::odom_commands, this, _1));
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry/unfiltered", 10);
    	//m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    	
    	m_out.open("/home/misha/out.txt");*/
    }

  private:

    void encodes_callback(const pivozavr_interfaces::msg::WheelInfo & msg)
    {
        double velocityFL = msg.fl;
        double velocityFR = msg.fr;
        double velocityML = msg.ml;
        double velocityMR = msg.mr;
        double velocityRL = msg.rl;
        double velocityRR = msg.rr;

        // Joint states block
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
                this->now().nanoseconds() - m_last_time.nanoseconds()));
        double dt = duration.seconds();
        m_last_time = this->now();

        positionFL += velocityFL*dt;
        positionFR += velocityFR*dt;
        positionML += velocityML*dt;
        positionMR += velocityMR*dt;
        positionRL += velocityRL*dt;
        positionRR += velocityRR*dt;

        joint_states_msg->position.push_back(positionFL);
        joint_states_msg->position.push_back(positionFR);
        joint_states_msg->position.push_back(positionML);
        joint_states_msg->position.push_back(positionMR);
        joint_states_msg->position.push_back(positionRL);
        joint_states_msg->position.push_back(positionRR);

        m_joint_states_pub->publish(std::move(joint_states_msg));

        return;

        /*// Odometry block
        if(m_first)
        {
            m_last_joint_positions[0] = motor_lf_position;
            m_last_joint_positions[1] = motor_rf_position;
            m_last_joint_positions[2] = motor_lr_position;
            m_last_joint_positions[3] = motor_rr_position;
            m_first = false;
            m_last_time = this->now();
            return;
        }
        m_diff_joint_positions[0] = motor_lf_position;// - m_last_joint_positions[0];
        m_diff_joint_positions[1] = motor_rf_position;// - m_last_joint_positions[1];
        m_diff_joint_positions[2] = motor_lr_position;// - m_last_joint_positions[2];
        m_diff_joint_positions[3] = motor_rr_position;// - m_last_joint_positions[3];*/
        /*m_last_joint_positions[0] = motor_lf_position;
        m_last_joint_positions[1] = motor_rf_position;
        m_last_joint_positions[2] = motor_lr_position;
        m_last_joint_positions[3] = motor_rr_position;*/


        /*double wheel_lf = m_diff_joint_positions[0];
        double wheel_rf = m_diff_joint_positions[1];
        double wheel_lr = m_diff_joint_positions[2];
        double wheel_rr = m_diff_joint_positions[3];

        FORWARD_DATA forw_data {wheel_lf, wheel_rf, wheel_lr, wheel_rr};

        odom_class->Update(forw_data);

//        RCLCPP_INFO(this->get_logger(), "%f", odom_class->posx);
//        RCLCPP_INFO(this->get_logger(), "%f", odom_class->posy);
//        RCLCPP_INFO(this->get_logger(), "%f", odom_class->rot);

        m_robot_pos[0] = odom_class->posx;
        m_robot_pos[1] = odom_class->posy;
        m_robot_pos[2] = odom_class->rot;

        rclcpp::Duration duration(rclcpp::Duration::from_nanoseconds(
                this->now().nanoseconds() - m_last_time.nanoseconds()));
        double step_time = duration.seconds();
        m_last_time = this->now();
        m_robot_vel[0] = odom_class->mv_after_update.VX;// / step_time;
        m_robot_vel[1] = odom_class->mv_after_update.VY;// / step_time;
        m_robot_vel[2] = odom_class->mv_after_update.WZ;// / step_time;

        // Odom publishing
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "base_footprint";
        odom_msg->header.stamp = this->now();

        odom_msg->pose.pose.position.x = m_robot_pos[0];
        odom_msg->pose.pose.position.y = m_robot_pos[1];
        odom_msg->pose.pose.position.z = 0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, m_robot_pos[2]);

        odom_msg->pose.pose.orientation.x = q.x();
        odom_msg->pose.pose.orientation.y = q.y();
        odom_msg->pose.pose.orientation.z = q.z();
        odom_msg->pose.pose.orientation.w = q.w();

        odom_msg->twist.twist.linear.x = m_robot_vel[0];
        odom_msg->twist.twist.linear.y = m_robot_vel[1];
        odom_msg->twist.twist.angular.z = m_robot_vel[2];
        m_out << std::setprecision(5) << m_robot_vel[0] << "," << m_robot_vel[1] << "," << m_robot_vel[2] << std::endl;

        geometry_msgs::msg::TransformStamped odom_tf;

        odom_tf.transform.translation.x = odom_msg->pose.pose.position.x;
        odom_tf.transform.translation.y = odom_msg->pose.pose.position.y;
        odom_tf.transform.translation.z = odom_msg->pose.pose.position.z;
        odom_tf.transform.rotation = odom_msg->pose.pose.orientation;

        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_footprint";
        odom_tf.header.stamp = this->now();

        m_odom_pub->publish(std::move(odom_msg));

        //m_tf_broadcaster->sendTransform(odom_tf);*/
    }


    void odom_commands(const geometry_msgs::msg::Twist & msg)
    {
        
//        RCLCPP_INFO(this->get_logger(), "%f", msg.linear.x);
//        RCLCPP_INFO(this->get_logger(), "%f", msg.linear.y);
//        RCLCPP_INFO(this->get_logger(), "%f", msg.angular.z);

        /*auto msg1 = telerobot_interfaces::msg::Motor();

        {
            INVERSE_DATA inv_data{msg.linear.x, msg.linear.y, msg.angular.z};
            FORWARD_DATA data = odom_class->GetOdometryINV(inv_data);
            //std::lock_guard<std::mutex> lock(m_mutex);*/
		/*if(m_is_body)
		{
			msg1.motor_lf = data.WFL * 10;
	            msg1.motor_rf = data.WFR * 10;
	            msg1.motor_lr = data.WRL * 10;
	            msg1.motor_rr = data.WRR * 10;
		}
		else
		{*/
	            //msg1.motor_lf = data.WFL;
	            //msg1.motor_rf = data.WFR;
	            //msg1.motor_lr = data.WRL;
	            //msg1.motor_rr = data.WRR;
	       // }
	//}

        //m_pub->publish(msg1);

    }

    double positionFL = 0;
    double positionFR = 0;
    double positionML = 0;
    double positionMR = 0;
    double positionRL = 0;
    double positionRR = 0;

    rclcpp::Subscription<pivozavr_interfaces::msg::WheelInfo>::SharedPtr m_encodes_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_states_pub;
    rclcpp::Time m_last_time;

    /*double m_diff_joint_positions[4] = {0};
    double m_last_joint_positions[4] = {0};
    bool m_first = true;

    double m_robot_pos[3] = {0,0,0};
    double m_robot_vel[3] = {0,0,0};
	
	bool m_is_body = false;

    rclcpp::Time m_start_time;
    rclcpp::Time m_last_time;
    std::unique_ptr<OdometryClass> odom_class;
    rclcpp::Publisher<telerobot_interfaces::msg::Motor>::SharedPtr m_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_odom_commands_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_body;
    //std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::ofstream m_out;*/
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
