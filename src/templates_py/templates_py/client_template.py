#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import ExampleService

class Client(Node):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.declare_parameter(name='length', value=0.0)
        self.declare_parameter(name='width', value=0.0)

        self.client = self.create_client(
            srv_type=ExampleService,
            srv_name='/example_service'
        )

        response = self.call(
            length=self.get_parameter(name='length').value,
            width=self.get_parameter(name='width').value
        )

        self.get_logger().info(message=f'Perimeter: {response.perimeter}')
        self.get_logger().info(message=f'Area: {response.area}')

    def call(self, length: float, width: float) -> ExampleService.Response:
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(message='Waiting for the server')

        request = ExampleService.Request()
        request.length = length
        request.width = width

        future = self.client.call_async(request=request)
        rclpy.spin_until_future_complete(node=self, future=future)

        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = Client('client')
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()