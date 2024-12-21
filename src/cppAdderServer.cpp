#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using Request = example_interfaces::srv::AddTwoInts::Request;
using Response = example_interfaces::srv::AddTwoInts::Response;

class adderServerNode : public rclcpp::Node
{
    public:
        adderServerNode() : Node("int_adder_server_node")
        {
            server_obj = this->create_service<example_interfaces::srv::AddTwoInts>(
                "two_ints_adder", std::bind(&adderServerNode::callbackFunc, this, _1, _2));
            
            RCLCPP_INFO(this->get_logger(), "Adder server has just been launched...");
        }

    private:
        void callbackFunc(const Request::SharedPtr request,
                      const Response::SharedPtr response)
        {
            response->sum = request->a + request->b;
            RCLCPP_INFO(this->get_logger(), "%ld + %ld = %ld", request->a, request->b, response->sum);
        }
        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_obj;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<adderServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}