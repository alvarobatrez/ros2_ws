#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/action/example_action.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class Client : public rclcpp::Node
{
    public:

    Client(std::string node_name) : Node(node_name)
    {

    }

    private:


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<Client>("client");
    rclcpp::spin(client);
}