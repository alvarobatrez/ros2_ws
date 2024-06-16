#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/action/example_action.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

typedef custom_interfaces::action::ExampleAction ExampleAction;
typedef rclcpp_action::ClientGoalHandle<ExampleAction> GoalHandle;

class Client : public rclcpp::Node
{
    public:

    Client(std::string node_name) : Node(node_name)
    {
        this->declare_parameter("goal", 10);
        int goal = this->get_parameter("goal").as_int();
        
        client = rclcpp_action::create_client<ExampleAction>
        (
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "/example_action"
        );

        send_goal(goal);
    }

    private:

    void send_goal(int goal)
    {
        while(!client->wait_for_action_server(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                return;
            }
            
            RCLCPP_WARN(this->get_logger(), "Waiting for the server");
        }

        auto goal_msg = ExampleAction::Goal();
        goal_msg.goal = goal;

        auto send_goal_options = rclcpp_action::Client<ExampleAction>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&Client::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = 
            std::bind(&Client::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&Client::result_callback, this, std::placeholders::_1);
        auto future = client->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(GoalHandle::SharedPtr goal_handle)
    {
        if (goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Goal canceled");
            rclcpp::shutdown();
        }
    }

    void feedback_callback
    (
        GoalHandle::SharedPtr,
        const std::shared_ptr<const ExampleAction::Feedback> feedback
    )
    {
        RCLCPP_INFO(this->get_logger(), "Feedback: %i", feedback->feedback);
    }

    void result_callback(const GoalHandle::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded: %s", result.result ? "true" : "false");
            break;      
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal failed with status: Canceled");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal failed with status: Aborted");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Goal failed with status: %i", result.code);
            break;
        }
    }

    rclcpp_action::Client<ExampleAction>::SharedPtr client;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<Client>("action_client");
    rclcpp::spin(action_client);
}