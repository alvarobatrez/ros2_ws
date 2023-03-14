#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import ExampleService

class ServerTemplate(Node):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.server = self.create_service(
            srv_type=ExampleService,
            srv_name='/example_service',
            callback=self.callback
        )

        self.get_logger().info(message='Server is ready')

    def callback(self, request: ExampleService.Request, response: ExampleService.Response) -> ExampleService.Response:
        response = ExampleService.Response()
        response.perimeter = 2 * (request.length + request.width)
        response.area = request.length * request.width

        self.get_logger().info(message='A service has been called')

        return response       

def main(args=None):
    rclpy.init(args=args)
    server_template = ServerTemplate(node_name='server_template')
    rclpy.spin(node=server_template)
    rclpy.shutdown()

if __name__ == '__main__':
    main()