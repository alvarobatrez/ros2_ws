#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/srv/example_service.hpp>

typedef custom_interfaces::srv::ExampleService ExampleService;

class Server : public rclcpp::Node
{
    public:

    Server(std::string node_name) : Node(node_name)
    {
        srv = this->create_service<ExampleService>
        (
            "/example_service",
            std::bind
            (
                &Server::callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

        RCLCPP_INFO(this->get_logger(), "The server is up");
    }

    private:

    void callback
    (
        const ExampleService::Request::SharedPtr req,
        const ExampleService::Response::SharedPtr res
    )
    {
        res->area = req->length * req->width;
        res->perimeter = 2 * (req->length + req->width);

        RCLCPP_INFO(this->get_logger(), "A service has been called");
    }

    rclcpp::Service<ExampleService>::SharedPtr srv;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto server = std::make_shared<Server>("server");
    rclcpp::spin(server);
    rclcpp::shutdown();

    return 0;
}