#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import ExampleService

class Server(Node):

    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        
        self.srv = self.create_service(
            srv_type=ExampleService,
            srv_name='/example_service',
            callback=self.callback
        )

        self.get_logger().info(message='The server is up')

    def callback(self, req, res):
        res.area = req.length * req.width
        res.perimeter = 2 * (req.length + req.width)

        self.get_logger().info(message='A service has been called')

        return res
    
def main(args=None):
    rclpy.init(args=args)
    server = Server(node_name='server')
    rclpy.spin(node=server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()