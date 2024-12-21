#include <iostream>
#include <stdexcept>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using Future = rclcpp::FutureReturnCode;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("int_adder_client_noop");
    
    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("two_ints_adder");
    // make sure service name is the same as server
    
    while (!client->wait_for_service(std::chrono::seconds(1)))
        RCLCPP_WARN(node->get_logger(), "Waiting for server become available...");

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();

    int aTemp, bTemp;
    //input type control
    while (true)
    {
        try
        {
            std::cout << "a: ";
            std::cin >> aTemp;

            if (std::cin.fail())
                throw std::invalid_argument("Invalid entry. Integers only!");

            break;
        } catch (const std::invalid_argument& e)
        {
            std::cerr << e.what() << std::endl;
            // clear invalid input and reset cin
            std::cin.clear(); 
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    while (true)
    {
        try
        {
            std::cout << "b: ";
            std::cin >> bTemp;

            if (std::cin.fail())
                throw std::invalid_argument("Invalid entry. Integers only!");

            break;
        } catch (const std::invalid_argument& e)
        {
            std::cerr << e.what() << std::endl;
            // clear invalid input and reset cin
            std::cin.clear(); 
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    request->a = aTemp;
    request->b = bTemp;

    auto futureObj = client->async_send_request(request);
    
    // WARNING !! rclcpp::executor::FutureReturnCode does not work, so it is omitted
    if (rclcpp::spin_until_future_complete(node, futureObj) == Future::SUCCESS)
        RCLCPP_INFO(node->get_logger(), "%d + %d = %d", (int)request->a, (int)request->b, (int)futureObj.get()->sum);
    
    else
        RCLCPP_ERROR(node->get_logger(), "Error while calling service!");
    
    rclcpp::shutdown();
    return 0;
}