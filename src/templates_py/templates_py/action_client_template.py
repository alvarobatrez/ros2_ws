#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from custom_interfaces.action import ExampleAction

class ActionClientTemplate(Node):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.declare_parameter(name='target', value=10)

        self.action_client = ActionClient(
            node=self,
            action_type=ExampleAction,
            action_name='/example_action'
        )

        self.send_goal(
            target=self.get_parameter(name='target').value
        )

    def send_goal(self, target: int):
        while not self.action_client.wait_for_server(1.0):
            self.get_logger().info('Waiting for the action server')

        goal = ExampleAction.Goal()
        goal.goal = target

        self.get_logger().info(f'Sending goal: {target}')

        future = self.action_client.send_goal_async(
            goal=goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(f'Feedback: {feedback.feedback.feedback}')

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle.accepted:
            self.get_logger().info('Goal accepted')
        else:
            self.get_logger().warn('Goal rejected')
            rclpy.shutdown()
            return
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal succeeded with result: {result.result}')
        else:
            self.get_logger().error(f'Goal failed with status: {status}')

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client_template = ActionClientTemplate('action_client_template')
    rclpy.spin(action_client_template)

if __name__ == '__main__':
    main()