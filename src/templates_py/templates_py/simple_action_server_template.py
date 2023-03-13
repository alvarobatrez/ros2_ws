#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_interfaces.action import ExampleAction
import time

class ActionServerTemplate(Node):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.action_server = ActionServer(
            node=self,
            action_type=ExampleAction,
            action_name='/example_action',
            execute_callback=self.execute_callback
        )

        self.get_logger().info('Action server is ready')

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request.goal
        result = ExampleAction.Result()
        feedback = ExampleAction.Feedback()
        success = False

        self.get_logger().info(f'Executing goal: {goal}')

        counter = 0
        max_number = 10

        while rclpy.ok():

            if counter > max_number:
                break

            if counter >= goal:
                success = True
                break

            counter += 1

            feedback.feedback = counter
            goal_handle.publish_feedback(feedback)

            time.sleep(1.0)

        result.result = success

        if success:
            self.get_logger().info('Goal succeed')
            goal_handle.succeed()
        else:
            self.get_logger().error('Goal aborted')
            goal_handle.abort()

        return result

def main(args=None):
    rclpy.init(args=args)
    simple_action_server_template = ActionServerTemplate('simple_action_server_template')
    rclpy.spin(simple_action_server_template)
    rclpy.shutdown()

if __name__ == '__main__':
    main()