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

        self.sock = None
        self.connect_tcp()

        # Create the service
        self.srv = self.create_service(JsonRpc, 'jsonrpc_service', self.handle_service)

        self.get_logger().info(f'[AUBO CLIENT] Ready to send TCP messages to {self.tcp_ip}:{self.tcp_port}')

    def connect_tcp(self):
        if self.sock:
            self.sock.close()
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.tcp_ip, self.tcp_port))
            self.get_logger().info(f'[TCP] Connected to {self.tcp_ip}:{self.tcp_port}')
        except Exception as e:
            self.get_logger().error(f'[TCP] Failed to connect: {e}')
            self.sock = None

    def handle_service(self, request, response):
        if not self.sock:
            self.get_logger().warn("[TCP] No connection, attempting to reconnect...")
            self.connect_tcp()
            if not self.sock:
                response.jsonrpc_response = "[ERROR] Failed to connect"
                return response

        try:
            # Send request
            self.get_logger().debug(f"[TCP] Sending: {request.jsonrpc_send}")
            self.sock.sendall(request.jsonrpc_send.encode('utf-8'))

            # Receive response
            data = self.sock.recv(4096).decode('utf-8')
            response.jsonrpc_response = data
            self.get_logger().debug(f"[TCP] Received: {data}")

        except Exception as e:
            self.get_logger().error(f"[TCP] Communication error: {e}")
            self.sock = None  # Force reconnect next time
            response.jsonrpc_response = f"[ERROR] {e}"

        return response

    def destroy_node(self):
        if self.sock:
            self.sock.close()
            self.get_logger().info("[TCP] Connection closed.")
        super().destroy_node()


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
