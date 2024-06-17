#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/action/example_action.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

typedef custom_interfaces::action::ExampleAction ExampleAction;
typedef rclcpp_action::ServerGoalHandle<ExampleAction> GoalHandle;

class Server : public rclcpp::Node
{
    public:

    Server(std::string node_name) : Node(node_name)
    {
        action_server = rclcpp_action::create_server<ExampleAction>
        (
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "/example_action",
            std::bind(&Server::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Server::handle_cancel, this, std::placeholders::_1),
            std::bind(&Server::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Action server is up");
    }

    private:

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing new goal");
        
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ExampleAction::Result>();
        auto feedback = std::make_shared<ExampleAction::Feedback>();
        bool success = false;
        bool cancel = false;

        int counter = 0;
        int max_num = 10;

        rclcpp::Rate rate(1.0);

        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling())
            {
                cancel = true;
                break;
            }

            if (counter == goal->goal)
            {
                success = true;
                break;
            }

            if (counter >=max_num)
            {
                break;
            }

            counter++;

            feedback->feedback = counter;
            goal_handle->publish_feedback(feedback);

            rate.sleep();
        }

        result->result = success;

        if (cancel)
        {
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Goal canceled");
        }
        else if (success)
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
        else
        {
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Goal aborted");
        }

        goal_queue.pop();

        if (!goal_queue.empty())
        {
            current_goal = goal_queue.front();
            RCLCPP_INFO(this->get_logger(), "Next goal put from the queue");
            std::thread{std::bind(&Server::execute, this, std::placeholders::_1), current_goal}.detach();
        }
    }

    rclcpp_action::GoalResponse handle_goal
    (
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ExampleAction::Goal> goal_request
    )
    {
        (void)uuid;
        if (goal_request->goal > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Incoming goal accepted");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Incoming goal rejected");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_WARN(this->get_logger(), "Canceling goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        goal_queue.push(goal_handle);
        if (goal_queue.size() == 1)
        {
            current_goal = goal_queue.front();
            std::thread{std::bind(&Server::execute, this, std::placeholders::_1), current_goal}.detach();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal put in the queue");
        }
    }

    rclcpp_action::Server<ExampleAction>::SharedPtr action_server;
    std::queue<std::shared_ptr<GoalHandle>> goal_queue;
    std::shared_ptr<GoalHandle> current_goal;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<Server>("action_server");
    rclcpp::spin(action_server);
    rclcpp::shutdown();

    return 0;
}