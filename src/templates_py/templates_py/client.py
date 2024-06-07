#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import ExampleService

class Client(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter(name='length', value=1.0)
        self.declare_parameter(name='width', value=1.0)

        length = self.get_parameter(name='length').value
        width = self.get_parameter(name='width').value

        self.call_server(length=length, width=width)

    def call_server(self, length, width):
        client = self.create_client(srv_type=ExampleService, srv_name='/example_service')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(message='Waiting for the server')

        req = ExampleService.Request()
        req.length = length
        req.width = width

        future = client.call_async(request=req)
        rclpy.spin_until_future_complete(node=self, future=future)
        res = future.result()

        self.get_logger().info(message=f'Area: {res.area}')
        self.get_logger().info(message=f'Perimeter: {res.perimeter}')

def main(args=None):
    rclpy.init(args=args)
    client = Client(node_name='client')
    rclpy.shutdown()

if __name__ == '__main__':
    main()