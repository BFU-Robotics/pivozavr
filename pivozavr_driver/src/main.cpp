#include <chrono>
#include <functional>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include <ModbusMaster.hpp>
#include "pivozavr_interfaces/msg/wheel_info.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class WheelDriver : public rclcpp::Node
{
  public:
    WheelDriver() : Node("pivozavr_driver")
    {
    	std::string device;
    	declare_parameter("dev", "");
    	get_parameter("dev", device);


        m_modbus = std::make_unique<robot::protocol::ModbusMaster>(device.c_str(), 115200);
        m_modbus->Setup();

        m_encoders_pub = this->create_publisher<pivozavr_interfaces::msg::WheelInfo>("wheel_velocities", 10);
		m_power_pub = this->create_publisher<std_msgs::msg::Bool>("power", 10);
        m_encoders_timer = this->create_wall_timer(10ms, std::bind(&WheelDriver::encoders_callback, this));
		m_power_timer = this->create_wall_timer(1s, std::bind(&WheelDriver::power_callback, this));

        m_wheel_commands_sub = this->create_subscription<pivozavr_interfaces::msg::WheelInfo>(
                "wheel_commands", 10, std::bind(&WheelDriver::wheel_commands, this, _1));

        m_servo_commands_sub = this->create_subscription<std_msgs::msg::Bool>(
                "servo_commands", 10, std::bind(&WheelDriver::servo_commands, this, _1));
                
        std::this_thread::sleep_for(std::chrono::seconds(4));

        std::lock_guard<std::mutex> lock(m_mutex);
            m_modbus->WriteMultiAnalogOutput(0x01, 0x0007,{
                    static_cast<int16_t>(1) // powerOn
            });
    }

    ~WheelDriver()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
            m_modbus->WriteMultiAnalogOutput(0x01, 0x0007,{
                    static_cast<int16_t>(0) // powerOff
            });
    }

  private:
    void wheel_commands(const pivozavr_interfaces::msg::WheelInfo & msg)
    {
        /*RCLCPP_INFO(this->get_logger(), "cmd %lf", msg.fl);
        RCLCPP_INFO(this->get_logger(), "%lf", msg.fr);
        RCLCPP_INFO(this->get_logger(), "%lf", msg.ml);
        RCLCPP_INFO(this->get_logger(), "%lf", msg.mr);
        RCLCPP_INFO(this->get_logger(), "%lf", msg.rl);
        RCLCPP_INFO(this->get_logger(), "%lf", msg.rr);*/

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_modbus->WriteMultiAnalogOutput(0x01, 0x0001,{
                    radsToRPM(msg.fr),       // FR
                    radsToRPM(msg.fl),        // FL
                    radsToRPM(msg.mr),
                    radsToRPM(msg.ml),
                    radsToRPM(msg.rr),
                    radsToRPM(msg.rl)
            });
        }
    }

    void servo_commands(const std_msgs::msg::Bool & msg)
    {
        /*if (msg.motor_rotate >= -45 && msg.motor_rotate <= 45)
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_modbus->WriteMultiAnalogOutput(0x01, 0x0006,{
                    static_cast<uint16_t>(msg.motor_rotate)
            });
        }
        if (msg.motor_pitch >= -20 && msg.motor_pitch <= 20)
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_modbus->WriteMultiAnalogOutput(0x01, 0x0005,{
                    static_cast<uint16_t>(msg.motor_pitch)
            });
        }*/
    }

    float rpmToRads(int16_t rpm)
    {
        return m_pi*rpm/30;
    }

    int16_t radsToRPM(float rads)
    {
        return static_cast<int16_t>(rads*30/m_pi);
    }

    void encoders_callback()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::vector<int16_t> data = m_modbus->ReadAnalogInput(0x01, 0x0001, 6);
		if(data.size() > 0)
		{
            /*std::cerr << "FR " << rpmToRads(data[0]) << std::endl;
            std::cerr << "FL " << rpmToRads(data[1]) << std::endl;
            std::cerr << "MR " << rpmToRads(data[2]) << std::endl;
            std::cerr << "ML " << rpmToRads(data[3]) << std::endl;
            std::cerr << "RR " << rpmToRads(data[4]) << std::endl;
            std::cerr << "RL " << rpmToRads(data[5]) << std::endl;*/

	    	auto encoders_msg = pivozavr_interfaces::msg::WheelInfo();
            encoders_msg.fl = rpmToRads(data[1]);
            encoders_msg.fr = rpmToRads(data[0]);
            encoders_msg.ml = rpmToRads(data[3]);
            encoders_msg.mr = rpmToRads(data[2]);
            encoders_msg.rl = rpmToRads(data[5]);
            encoders_msg.rr = rpmToRads(data[4]);
            m_encoders_pub->publish(encoders_msg);
		}
        //std::cerr << "===" << std::endl;
    }

	void power_callback()
    {
        /*std::lock_guard<std::mutex> lock(m_mutex);  
		std::vector<int16_t> data = m_modbus->ReadAnalogInput(0x01, 0x0009, 2);
		if(data.size() > 0)
    	{*/
        	/*auto power_msg = telerobot_interfaces::msg::Power();
           	power_msg.voltage = data[0]/100.f;
           	power_msg.percent = data[1]/100.f;
           	//m_power_pub->publish(power_msg);*/
    	//}
    }


    std::mutex m_mutex;
    std::unique_ptr<robot::protocol::ModbusMaster> m_modbus;
    rclcpp::TimerBase::SharedPtr m_encoders_timer;
    rclcpp::TimerBase::SharedPtr m_power_timer;
	rclcpp::Publisher<pivozavr_interfaces::msg::WheelInfo>::SharedPtr m_encoders_pub;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_power_pub;
    rclcpp::Subscription<pivozavr_interfaces::msg::WheelInfo>::SharedPtr m_wheel_commands_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_servo_commands_sub;
    const float m_pi = acos(-1);
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelDriver>());
  rclcpp::shutdown();
  return 0;
}
