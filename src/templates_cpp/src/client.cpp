#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/srv/example_service.hpp>

typedef custom_interfaces::srv::ExampleService ExampleService;

class Client : public rclcpp::Node
{
    public:

    Client(std::string node_name) : Node(node_name) {}

    void send_request(double length, double width)
    {
        rclcpp::Client<ExampleService>::SharedPtr client = this->create_client<ExampleService>("/example_service");

        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                return;
            }
            
            RCLCPP_WARN(this->get_logger(), "Waiting for the server");
        }
        
        std::shared_ptr<ExampleService::Request> req = std::make_shared<ExampleService::Request>();
        req->length = length;
        req->width = width;

        rclcpp::Client<ExampleService>::FutureAndRequestId future = client->async_send_request(req);

        if (rclcpp::spin_until_future_complete(shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            std::shared_ptr<ExampleService::Response> res = future.get();
            RCLCPP_INFO(this->get_logger(), "Area: %f", res->area);
            RCLCPP_INFO(this->get_logger(), "Perimeter: %f", res->perimeter);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }

    }

    double length, width;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Client> client = std::make_shared<Client>("client");
    
    client->declare_parameter("length", 0.0);
    client->declare_parameter("width", 0.0);

    double length = client->get_parameter("length").as_double();
    double width = client->get_parameter("width").as_double();

    client->send_request(length, width);

    rclcpp::shutdown();
    
    return 0;
}