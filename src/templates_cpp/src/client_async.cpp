#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/srv/example_service.hpp>

typedef custom_interfaces::srv::ExampleService ExampleService;

class Client : public rclcpp::Node
{
    public:

    Client(std::string node_name) : Node(node_name)
    {
        this->declare_parameter("length", 0.0);
        this->declare_parameter("width", 0.0);

        double length = this->get_parameter("length").as_double();
        double width = this->get_parameter("width").as_double();

        thread.push_back(std::thread(std::bind(&Client::send_request, this, length, width)));
    }

    ~Client()
    {
        for (auto &th : thread)
        {
            if (th.joinable())
            {
                th.join();
            }
        }
    }

    private:

    void send_request(double length, double width)
    {
        auto client = this->create_client<ExampleService>("/example_service");

        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                return;
            }
            
            RCLCPP_WARN(this->get_logger(), "Waiting for the server");
        }
        
        auto req = std::make_shared<ExampleService::Request>();
        req->length = length;
        req->width = width;

        auto future = client->async_send_request(req);

        try
        {
            auto res = future.get();
            RCLCPP_INFO(this->get_logger(), "Area: %f", res->area);
            RCLCPP_INFO(this->get_logger(), "Perimeter: %f", res->perimeter);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }

        rclcpp::shutdown();

    }

    std::vector<std::thread> thread;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<Client>("client");
    rclcpp::spin(client);
    
    return 0;
}