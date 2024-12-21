#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/hardware_stat.hpp"

class HardwareStatusPublisher : public rclcpp::Node
{
public:
    HardwareStatusPublisher() : Node("cppHwStatPubNode")
    {
        pubObj = this->create_publisher<custom_interfaces::msg::HardwareStat>("hw_stat", 10);
        timerObj = this->create_wall_timer(std::chrono::seconds(1),
                        std::bind(&HardwareStatusPublisher::publishCallback, this));
        RCLCPP_INFO(this->get_logger(), "Hardware diagnostics is running...");
    }
private:
    rclcpp::Publisher<custom_interfaces::msg::HardwareStat>::SharedPtr pubObj;
    rclcpp::TimerBase::SharedPtr timerObj;

    void publishCallback()
    {
        auto msg = custom_interfaces::msg::HardwareStat();
        msg.motortemp = 67;
        msg.is_motor_ready = false;
        msg.debugmsg = "Engine failure!";
        pubObj->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
