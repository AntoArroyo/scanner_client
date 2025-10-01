import rclpy
from rclpy.node import Node
import subprocess
import requests
import argparse
import statistics
import ast
from collections import deque

from visualization_msgs.msg import Marker  

from .wifi_scanner import WiFiScanner

BUFFER_SIZE = 5
class WifiScannerNode(Node):
    def __init__(self, device_id, server_ip, map_name='LAB', timer=5.0, use_median=True, alpha=0.2):
        super().__init__('wifi_scanner_node')

        self.marker_pub = self.create_publisher(Marker, 'estimated_marker', 10)
        self.timer = self.create_timer(float(timer), self.scan_and_send) 

        self.map_name = map_name
        self.device_id = device_id
        self.api_url = f'http://{server_ip}:8000/localize/{self.map_name}'
        self.wifi_scanner = WiFiScanner(self.get_logger())
        
        # Buffer to store last N positions
        self.position_buffer = deque(maxlen=BUFFER_SIZE)
        self.use_median = use_median

        # Exponential smoothing state
        self.alpha = alpha
        self.filtered_pos = None  # (x,y,z)

    def scan_and_send(self):
        try:
            wifi_data = self.wifi_scanner.scan_wifi()
            self.get_logger().info(f"Wifi data processed -- \n{wifi_data}\n")
            self.send_to_localization_server(wifi_data)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"WiFi scan failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def send_to_localization_server(self, wifi_data):
        payload = {
            "device_id": self.device_id,
            "wifi_signals": wifi_data
        }
        headers = {'Content-Type': 'application/json'}

        try:
            response = requests.post(self.api_url, headers=headers, json=payload)
        
            if response.status_code == 200:
                est_pos = response.json().get("EstimatedPosition")
                if est_pos:
                    # Convert string to tuple safely
                    tupla = tuple(map(float, ast.literal_eval(est_pos))) 
                    self.position_buffer.append(tupla)
                    
                    self.get_logger().info(f"Localization result: {tupla}")

                    # Decide which position to start with
                    if self.use_median and self.position_buffer:
                        columnas = list(zip(*self.position_buffer))
                        base_pos = tuple(statistics.median(col) for col in columnas)
                        self.get_logger().info(f"Buffer median result: {base_pos}")
                    else:
                        base_pos = tupla

                    # Apply exponential smoothing
                    if self.filtered_pos is None:
                        self.filtered_pos = base_pos
                    else:
                        self.filtered_pos = tuple(
                            self.alpha * b + (1 - self.alpha) * f
                            for b, f in zip(base_pos, self.filtered_pos)
                        )

                    self.get_logger().info(f"Smoothed result: {self.filtered_pos}")
                    self.publish_marker(self.filtered_pos, ns='estimated_position_smoothed')
            else:
                self.get_logger().error(f"Failed to get localization. Status: {response.status_code} | {response.text}")

        except requests.RequestException as e:
            self.get_logger().error(f"HTTP request failed: {e}")

    def publish_marker(self, position, ns='estimated_position'):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)
        self.get_logger().info(f"Published Marker at {position}")


def main(args=None):
    parser = argparse.ArgumentParser(description='Wi-Fi Scanner Node')
    parser.add_argument('--device_id', required=True, help='Device ID (e.g., turtlebot3_LAB)')
    parser.add_argument('--server_ip', required=True, help='Map server IP or hostname (e.g., 192.168.1.100)')
    parser.add_argument('--map_name', required=True, help='Map name to use for localization')
    parser.add_argument('--timer', default=5.0, help='Timer for scan and send (default 5 seconds)')
    parser.add_argument('--median', action='store_true', help='Use median filter over last positions')
    parser.add_argument('--alpha', type=float, default=0.2, help='Smoothing factor for exponential filter (0-1)')

    parsed_args, _ = parser.parse_known_args()  # allows ROS args to pass through

    rclpy.init(args=args)
    node = WifiScannerNode(
        device_id=parsed_args.device_id,
        server_ip=parsed_args.server_ip,
        map_name=parsed_args.map_name,
        timer=parsed_args.timer,
        use_median=parsed_args.median,
        alpha=parsed_args.alpha
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
