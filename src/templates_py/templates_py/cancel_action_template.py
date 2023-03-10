#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from action_msgs.srv import CancelGoal

class CancelActionTemplate(Node):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.client = self.create_client(
            srv_type=CancelGoal,
            srv_name='/example_action/_action/cancel_goal'
        )

        self.cancel()

    def cancel(self):
        while not self.client.wait_for_service(1.0):
            self.get_logger().warn(message='Waiting for the action server')

        self.get_logger().warn('Canceling goal')

        request = CancelGoal.Request()

        future = self.client.call_async(request=request)
        rclpy.spin_until_future_complete(node=self, future=future)

        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().warn('Goal successfully canceled')
        else:
            self.get_logger().error('Goal failed to cancel')

    
def main(args=None):
    rclpy.init(args=args)
    cancel_action_template = CancelActionTemplate('cancel_action_template')
    rclpy.shutdown()

if __name__ == '__main__':
    main()