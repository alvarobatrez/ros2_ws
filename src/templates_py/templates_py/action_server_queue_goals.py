#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.action import ExampleAction
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import collections
import threading

class Server(Node):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)

        self.goal_queue = collections.deque()
        self.goal_queue_lock = threading.Lock()
        self.current_goal = None

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

        self.get_logger().info(message='Action server is up')

    def execute_callback(self, goal_handle):
        try:
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
        
        finally:
            with self.goal_queue_lock:
                try:
                    self.current_goal = self.goal_queue.popleft()
                    self.get_logger().info(message='Next goal pulled from the queue')
                    self.current_goal.execute()
                except IndexError:
                    self.current_goal = None

    def goal_callback(self, goal_request):
        if goal_request.goal > 0:
            self.get_logger().info(message='Incoming goal accepted')
            return GoalResponse.ACCEPT
        else:
            self.get_logger().warn(message='Incoming goal rejected')
            return GoalResponse.REJECT
        
    def handle_accepted_callback(self, goal_handle):
        with self.goal_queue_lock:
            
            if self.current_goal is not None:
                self.goal_queue.append(goal_handle)
                self.get_logger().info(message='Goal put in the queue')
            else:
                self.current_goal = goal_handle
                self.current_goal.execute()


    def cancel_callback(self, goal_handle):
        self.get_logger().warn('Canceling goal')
        return CancelResponse.ACCEPT
    
def main(args=None):
    rclpy.init(args=args)
    action_server = Server(node_name='action_server')
    rclpy.spin(node=action_server, executor=MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()