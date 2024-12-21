#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class cppAdderClientOOP : public rclcpp::Node
{
public:
    cppAdderClientOOP() : Node("int_adder_client_oop")
    {
        int aTemp, bTemp;
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
                std::cin.clear(); 
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }

        thr1 = std::thread(std::bind(&cppAdderClientOOP::makeRequest, this, aTemp, bTemp));

    }
    void makeRequest(int a, int b)
    {
        auto client_obj = this->create_client<example_interfaces::srv::AddTwoInts>("two_ints_adder");
        while (!client_obj->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for server become available...");

        auto request_obj = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        
        request_obj->a = a;
        request_obj->b = b;

        auto future_obj = client_obj->async_send_request(request_obj);

        try
        {
            auto response_obj = future_obj.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a, b, (int)response_obj->sum);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed!");
        }
    }

private:
    std::thread thr1;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cppAdderClientOOP>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}