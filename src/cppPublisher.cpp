#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class cppRoboStation : public rclcpp::Node
{
public:
    cppRoboStation() : Node("cppRoboNode")
    {
        publisherObj = this->create_publisher<example_interfaces::msg::String>("RoboNews", 10);
        timerObj = this->create_wall_timer(std::chrono::milliseconds(780),
                                           std::bind(&cppRoboStation::publishNews, this));

        RCLCPP_INFO(this->get_logger(), "Robo News is now live!");
    }

private:
    void publishNews()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Welcome to the Robo News Radio!");
        publisherObj->publish(msg);
    }
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisherObj;
    rclcpp::TimerBase::SharedPtr timerObj;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cppRoboStation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}