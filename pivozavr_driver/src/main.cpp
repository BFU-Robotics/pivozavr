#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <ModbusMaster.hpp>

#include "rclcpp/rclcpp.hpp"
#include "pivozavr_interfaces/msg/wheel_info.hpp"
#include "pivozavr_interfaces/msg/aux_info.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PivozavrDriver : public rclcpp::Node
{
  public:
    PivozavrDriver() : Node("pivozavr_driver")
    {
        // Порт для связи с микроконтролером робота
    	std::string device;
    	declare_parameter("dev", "/dev/ttyUSB0");
    	get_parameter("dev", device);

        // Настройка Modbus
        m_modbus = std::make_unique<robot::protocol::ModbusMaster>(device.c_str(), 115200);
        m_modbus->Setup();

        m_velocityPub = this->create_publisher<pivozavr_interfaces::msg::WheelInfo>("wheel_velocities", 10);
		m_auxInfoPub = this->create_publisher<pivozavr_interfaces::msg::AuxInfo>("aux_info", 10);
        m_velocityTimer = this->create_wall_timer(10ms, std::bind(&PivozavrDriver::velocityСallback, this));
		m_auxInfoTimer = this->create_wall_timer(1s, std::bind(&PivozavrDriver::auxInfoCallback, this));
        m_wheelCommandsSub = this->create_subscription<pivozavr_interfaces::msg::WheelInfo>(
                "wheel_commands", 10, std::bind(&PivozavrDriver::wheelСommands, this, _1));

        // Задержка для включения микроконтроллера робота 
        std::this_thread::sleep_for(std::chrono::seconds(4));

        // Отправка команды на включение мостов
        std::lock_guard<std::mutex> lock(m_mutex);
        m_modbus->WriteMultiAnalogOutput(0x01, 0x0007,{ static_cast<int16_t>(1) });
    }

    ~PivozavrDriver()
    {
        // Отправка команды на отключение мостов
        std::lock_guard<std::mutex> lock(m_mutex);
        m_modbus->WriteMultiAnalogOutput(0x01, 0x0007,{ static_cast<int16_t>(0) });
    }

  private:
    void wheelСommands(const pivozavr_interfaces::msg::WheelInfo & msg)
    {
        /*
        // Отладочный вывод
        RCLCPP_INFO(this->get_logger(), "cmd %lf", msg.fl);
        RCLCPP_INFO(this->get_logger(), "%lf", msg.fr);
        RCLCPP_INFO(this->get_logger(), "%lf", msg.ml);
        RCLCPP_INFO(this->get_logger(), "%lf", msg.mr);
        RCLCPP_INFO(this->get_logger(), "%lf", msg.rl);
        RCLCPP_INFO(this->get_logger(), "%lf", msg.rr);
        */

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_modbus->WriteMultiAnalogOutput(0x01, 0x0001,{
                    radsToRPM(msg.fr),
                    radsToRPM(msg.fl),     
                    radsToRPM(msg.mr),
                    radsToRPM(msg.ml),
                    radsToRPM(msg.rr),
                    radsToRPM(msg.rl)
            });
        }
    }

    void velocityСallback()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::vector<int16_t> data = m_modbus->ReadAnalogInput(0x01, 0x0001, 6);
		if(data.size() > 0)
		{
            /*
            // Отладочный вывод
            std::cerr << "FR " << rpmToRads(data[0]) << std::endl;
            std::cerr << "FL " << rpmToRads(data[1]) << std::endl;
            std::cerr << "MR " << rpmToRads(data[2]) << std::endl;
            std::cerr << "ML " << rpmToRads(data[3]) << std::endl;
            std::cerr << "RR " << rpmToRads(data[4]) << std::endl;
            std::cerr << "RL " << rpmToRads(data[5]) << std::endl;
            */

	    	auto velocityMsg = pivozavr_interfaces::msg::WheelInfo();
            velocityMsg.fl = rpmToRads(data[1]);
            velocityMsg.fr = rpmToRads(data[0]);
            velocityMsg.ml = rpmToRads(data[3]);
            velocityMsg.mr = rpmToRads(data[2]);
            velocityMsg.rl = rpmToRads(data[5]);
            velocityMsg.rr = rpmToRads(data[4]);
            m_velocityPub->publish(velocityMsg);
		}
    }

	void auxInfoCallback()
    {
        std::lock_guard<std::mutex> lock(m_mutex);  
		std::vector<int16_t> data = m_modbus->ReadAnalogInput(0x01, 0x0007, 6);
		if(data.size() > 0)
    	{
        	auto auxInfoMsg = pivozavr_interfaces::msg::AuxInfo();
           	auxInfoMsg.bat_bridge1 = data[0]/100.f;
           	auxInfoMsg.bat_bridge2 = data[2]/100.f;
            auxInfoMsg.bat_bridge3 = data[4]/100.f;
            auxInfoMsg.temp_bridge3 = data[1]/10.f;
            auxInfoMsg.temp_bridge3 = data[3]/10.f;
            auxInfoMsg.temp_bridge3 = data[5]/10.f;
           	m_auxInfoPub->publish(auxInfoMsg);
    	}
    }

    float rpmToRads(int16_t rpm)
    {
        return m_pi*rpm/30;
    }

    int16_t radsToRPM(float rads)
    {
        return static_cast<int16_t>(rads*30/m_pi);
    }

private:
    std::mutex m_mutex;
    std::unique_ptr<robot::protocol::ModbusMaster> m_modbus;
    rclcpp::TimerBase::SharedPtr m_velocityTimer;
    rclcpp::TimerBase::SharedPtr m_auxInfoTimer;
	rclcpp::Publisher<pivozavr_interfaces::msg::WheelInfo>::SharedPtr m_velocityPub;
	rclcpp::Publisher<pivozavr_interfaces::msg::AuxInfo>::SharedPtr m_auxInfoPub;
    rclcpp::Subscription<pivozavr_interfaces::msg::WheelInfo>::SharedPtr m_wheelCommandsSub;
    const float m_pi = acos(-1);
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PivozavrDriver>());
  rclcpp::shutdown();
  return 0;
}
