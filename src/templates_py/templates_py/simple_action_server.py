#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.action import ExampleAction
from rclpy.action import ActionServer
import time

class Server(Node):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)

        self.action_server = ActionServer(
            node=self,
            action_type=ExampleAction,
            action_name='/example_action',
            execute_callback=self.execute_callback
        )

        self.get_logger().info(message='Action server is up')

    def execute_callback(self, goal_handle):
        self.get_logger().info(message='Executing new goal')
        goal = goal_handle.request.goal
        result = ExampleAction.Result()
        feedback = ExampleAction.Feedback()
        success = False
        cancel = False

        counter = 0
        max_num = 10

        while rclpy.ok():
            
            if goal_handle.is_cancel_requested:
                cancel = True
                break

            if counter == goal:
                success = True
                break

            if counter >= max_num:
                break

            counter += 1

            feedback.feedback = counter
            goal_handle.publish_feedback(feedback)

            time.sleep(1.0)

        result.result = success

        if cancel:
            goal_handle.canceled()
            self.get_logger().warn(message='Goal canceled')
        elif success:
            goal_handle.succeed()
            self.get_logger().info(message='Goal succeeded')
        else:
            goal_handle.abort()
            self.get_logger().error(message='Goal aborted')

        return result
    
def main(args=None):
    rclpy.init(args=args)
    action_server = Server(node_name='action_server')
    rclpy.spin(node=action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()