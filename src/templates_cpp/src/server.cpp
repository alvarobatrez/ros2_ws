#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/srv/example_service.hpp>

class Server : public rclcpp::Node
{
    public:

    Server(std::string node_name) : Node(node_name)
    {
        
    }

    private:

    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Server> server = std::make_shared<Server>("server");
    rclcpp::spin(server);
    rclcpp::shutdown();

    return 0;
}