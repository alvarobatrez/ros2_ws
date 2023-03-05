#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class PublisherTemplate : public rclcpp::Node
{
    public:

    PublisherTemplate() : Node("publisher_template")
    {
        pub = this->create_publisher<std_msgs::msg::String>(
            "/example_topic",
            10
        );

        timer = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&PublisherTemplate::publish, this)
        );
    }

    private:
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;

    void publish()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "News";

        RCLCPP_INFO(this->get_logger(), "Transmitting: %s", msg.data.c_str());

        pub->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto publisher_template = std::make_shared<PublisherTemplate>();
    rclcpp::spin(publisher_template);
    rclcpp::shutdown();

    return 0;
}