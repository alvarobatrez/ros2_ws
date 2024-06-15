#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.action import ExampleAction
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

class Client(Node):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)

        self.declare_parameter(name='goal', value=10)

        goal = self.get_parameter(name='goal').value

        self.action_client = ActionClient(
            node=self,
            action_type=ExampleAction,
            action_name='/example_action'
        )

        self.send_goal(goal)

    def send_goal(self, goal):
        goal_msg = ExampleAction.Goal()
        goal_msg.goal = goal

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(message='Waiting for the action server')

        future = self.action_client.send_goal_async(
            goal=goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(callback=self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if goal_handle.accepted:
            self.get_logger().info(message='Goal accepted')
        else:
            self.get_logger().warn(message='Goal rejected')
            rclpy.shutdown()

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        terminal_states = {
            4: 'Succeeded',
            5: 'Canceled',
            6: 'Aborted'
        }

        result = future.result().result.result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(message=f'Goal succeeded: {result}')
        else:
            self.get_logger().error(message=f'Goal failed with status: {terminal_states[status]}')
        
        rclpy.shutdown()

    def feedback_callback(self, feedback):
        self.get_logger().info(message=f'Feedback: {feedback.feedback.feedback}')
    
def main(args=None):
    rclpy.init(args=args)
    action_client =Client(node_name='action_client')
    rclpy.spin(node=action_client)

if __name__ == '__main__':
    main()