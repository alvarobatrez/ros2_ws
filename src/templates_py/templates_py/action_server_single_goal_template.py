#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from custom_interfaces.action import ExampleAction
import threading
import time

class ActionServerTemplate(Node):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.goal_handle = None
        self.goal_lock = threading.Lock()

        self.action_server = ActionServer(
            node=self,
            action_type=ExampleAction,
            action_name='/example_action',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Action server is ready')

    def execute_callback(self, goal_handle):
        goal = goal_handle.request.goal
        result = ExampleAction.Result()
        feedback = ExampleAction.Feedback()
        success = False
        cancel = False

        self.get_logger().info(f'Executing goal: {goal}')

        counter = 0
        max_number = 10

        while rclpy.ok():

            if not goal_handle.is_active:
                return result

            if goal_handle.is_cancel_requested:
                cancel = True
                break

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

        if cancel:
            self.get_logger().warn('Goal canceled')
            goal_handle.canceled()
        elif success:
            self.get_logger().info('Goal succeed')
            goal_handle.succeed()
        else:
            self.get_logger().error('Goal aborted')
            goal_handle.abort()

        return result
    
    def goal_callback(self, goal_request):
        if goal_request.goal < 1:
            self.get_logger().warn('Goal request rejected')
            return GoalResponse.REJECT
        else:
            self.get_logger().info('Goal request accepted')
            return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().warn('Aborting previous goal')
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('Cancel request accepted')
        return CancelResponse.ACCEPT

def main(args=None):
    rclpy.init(args=args)
    action_server_single_goal_template = ActionServerTemplate('action_server_single_goal_template')
    rclpy.spin(action_server_single_goal_template, executor=MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()