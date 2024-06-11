#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Server(Node):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)
    
def main(args=None):
    rclpy.init(args=args)
    action_server = Server(node_name='action_server')
    rclpy.spin(node=action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()