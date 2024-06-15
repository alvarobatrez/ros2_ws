#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.action import ExampleAction
from rclpy.action import ActionClient

class Client(Node):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)

        
    
def main(args=None):
    rclpy.init(args=args)
    action_client =Client(node_name='action_client')
    rclpy.spin(node=action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()