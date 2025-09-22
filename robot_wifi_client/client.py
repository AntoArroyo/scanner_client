import rclpy
from rclpy.node import Node
import subprocess
import requests
import argparse
import statistics
import ast

from visualization_msgs.msg import Marker  

from .wifi_scanner import WiFiScanner

MEDIAN = False


class WifiScannerNode(Node):
    def __init__(self, device_id, server_ip, map_name='LAB', timer=5.0):
        super().__init__('wifi_scanner_node')

        self.marker_pub = self.create_publisher(Marker, 'estimated_marker', 10)

        self.timer = self.create_timer(float(timer), self.scan_and_send) 
        self.map_name = map_name
        self.device_id = device_id
        self.api_url = f'http://{server_ip}:8000/localize/{self.map_name}'
        self.wifi_scanner = WiFiScanner(self.get_logger())
        self.position_buffer = []

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
                    tupla = tuple(map(float, ast.literal_eval(est_pos))) 
                
                if len(self.position_buffer) < 11:   
                    self.position_buffer.append(tupla)
                else:
                    self.position_buffer = []

                self.get_logger().info(f"Localization result: {tupla}")
                # Publish as Point
                if est_pos and not MEDIAN: 
                    marker = Marker()
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.header.frame_id = 'map'  # same as your map frame
                    marker.ns = 'estimated_position'
                    marker.id = 0
                    marker.type = Marker.SPHERE  # just a point
                    marker.action = Marker.ADD
                    marker.pose.position.x = tupla[0]
                    marker.pose.position.y = tupla[1]
                    marker.pose.position.z = tupla[2]
                    marker.pose.orientation.w = 1.0  # neutral orientation
                    marker.scale.x = 0.35
                    marker.scale.y = 0.35
                    marker.scale.z = 0.35
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0

                    self.marker_pub.publish(marker)
                    self.get_logger().info(f"Published Marker pos at {tupla}")
                
                if self.position_buffer and MEDIAN:
                    columnas = list(zip(*self.position_buffer))
                    # Median of each column
                    mediana = tuple(statistics.median(col) for col in columnas)
                    self.get_logger().info(f"Buffer result: {mediana}")

                    # Publish as Point
                    marker = Marker()
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.header.frame_id = 'map'  # same as your map frame
                    marker.ns = 'estimated_position'
                    marker.id = 0
                    marker.type = Marker.SPHERE  # just a point
                    marker.action = Marker.ADD
                    marker.pose.position.x = mediana[0]
                    marker.pose.position.y = mediana[1]
                    marker.pose.position.z = mediana[2]
                    marker.pose.orientation.w = 1.0  # neutral orientation
                    marker.scale.x = 0.35
                    marker.scale.y = 0.35
                    marker.scale.z = 0.35
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0

                    self.marker_pub.publish(marker)
                    self.get_logger().info(f"Published Marker md at {mediana}")
            else:
                self.get_logger().error(f"Failed to get localization. Status: {response.status_code} | {response.text}")

        except requests.RequestException as e:
            self.get_logger().error(f"HTTP request failed: {e}")


def main(args=None):
    parser = argparse.ArgumentParser(description='Wi-Fi Scanner Node')
    parser.add_argument('--device_id', required=True, help='Device ID (e.g., turtlebot3_LAB)')
    parser.add_argument('--server_ip', required=True, help='Map server IP or hostname (e.g., 192.168.1.100)')
    parser.add_argument('--map_name', required=True, help='Map name to use for localization')
    parser.add_argument('--timer', default=5.0, help='Timer for scan and send (default 5 seconds)')

    parsed_args, _ = parser.parse_known_args()  # allows ROS args to pass through

    rclpy.init(args=args)
    node = WifiScannerNode(
        device_id=parsed_args.device_id,
        server_ip=parsed_args.server_ip,
        map_name=parsed_args.map_name,
        timer=parsed_args.timer
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
