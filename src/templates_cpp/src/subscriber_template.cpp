#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SubscriberTemplate : public rclcpp::Node
{
    public:

    SubscriberTemplate() : Node("subscriber_template")
    {
        sub = this->create_subscription<std_msgs::msg::String>("example_topic", 10,
            std::bind(&SubscriberTemplate::callback, this, std::placeholders::_1)
        );
    }

    private:

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;

    void callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Receiving: %s", msg->data.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto subscriber_template = std::make_shared<SubscriberTemplate>();
    rclcpp::spin(subscriber_template);
    rclcpp::shutdown();

    return 0;
}