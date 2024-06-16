#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Receiver : public rclcpp::Node
{
    public:

    Receiver(std::string node_name) : Node(node_name)
    {
        sub = this->create_subscription<std_msgs::msg::String>
        (
            "/example_topic",
            10,
            std::bind(&Receiver::callback, this, std::placeholders::_1)
        );
    }

    private:

    void callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Receiving: %s", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto receiver = std::make_shared<Receiver>("receiver");
    rclcpp::spin(receiver);
    rclcpp::shutdown();

    return 0;
}