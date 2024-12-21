#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class cppSubsNode : public rclcpp::Node
{
public:
    cppSubsNode () : Node("cppSubsNode")
    {
        subsObj = this->create_subscription<example_interfaces::msg::String>("RoboNews", 10,
                std::bind(&cppSubsNode::callbackSubscriber, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Smartphone listener has been started.");
    }

private:
    void callbackSubscriber(const example_interfaces::msg::String::SharedPtr message)
    {
        RCLCPP_INFO(this->get_logger(), "%s", message->data.c_str());
    }
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subsObj;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<cppSubsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}