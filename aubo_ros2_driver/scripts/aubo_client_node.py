#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aubo_msgs.srv import JsonRpc

import socket


class TcpClientService(Node):
    def __init__(self):
        super().__init__('aubo_client')

        # Declare and get parameters
        self.declare_parameter('tcp_client.ip', '127.0.0.1')
        self.declare_parameter('tcp_client.port', 30004)

        self.tcp_ip = self.get_parameter('tcp_client.ip').get_parameter_value().string_value
        self.tcp_port = self.get_parameter('tcp_client.port').get_parameter_value().integer_value

        # Create the service
        self.srv = self.create_service(JsonRpc, 'jsonrpc_service', self.handle_service)

        self.get_logger().info(f'[AUBO CLIENT] Ready to send TCP messages to {self.tcp_ip}:{self.tcp_port}')

    def handle_service(self, request, response):
        try:
            self.get_logger().info(f"[TCP] Connecting to {self.tcp_ip}:{self.tcp_port} ...")
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(5.0)  # Timeout for safety
                sock.connect((self.tcp_ip, self.tcp_port))
                self.get_logger().info(f"[TCP] Sending: {request.jsonrpc_send}")
                sock.sendall(request.jsonrpc_send.encode('utf-8'))

                # Receive response
                data = sock.recv(4096).decode('utf-8')  # Increase buffer if needed
                response.jsonrpc_response = data
                self.get_logger().info(f"[TCP] Received: {data}")

        except Exception as e:
            error_msg = f"TCP communication error: {e}"
            self.get_logger().error(error_msg)
            response.jsonrpc_response = f"[ERROR] {error_msg}"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TcpClientService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down AUBO TCP Client Node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
