#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Transmitter : public rclcpp::Node
{
    public:

    Transmitter(std::string node_name) : Node(node_name)
    {
        this->declare_parameter("period", 1.0);
        period = this->get_parameter("period").as_double();
        
        pub = this->create_publisher<std_msgs::msg::String>("/example_topic", 10);
        timer = this->create_wall_timer
        (
            std::chrono::milliseconds(static_cast<int>(period*1000)),
            std::bind(&Transmitter::publish_callback, this)
        );
    }

    private:

    void publish_callback()
    {
        std_msgs::msg::String msg;
        msg.data = "news";
        pub->publish(msg);

        // RCLCPP_INFO(this->get_logger(), "Transmitting: %s", msg.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;
    double period;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto transmitter = std::make_shared<Transmitter>("transmitter");
    rclcpp::spin(transmitter);
    rclcpp::shutdown();

    return 0;
}