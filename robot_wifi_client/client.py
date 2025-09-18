import rclpy
from rclpy.node import Node
import subprocess
import requests
import argparse
from .wifi_scanner import WiFiScanner

class WifiScannerNode(Node):
    def __init__(self, device_id, server_ip, map_name='LAB', timer='5.0'):
        super().__init__('wifi_scanner_node')
        self.timer = self.create_timer(timer, self.scan_and_send) 
        self.map_name = map_name
        self.device_id = device_id
        self.api_url = f'http://{server_ip}:8000/localize/{self.map_name}'
        self.wifi_scanner = WiFiScanner(self.get_logger())

    def scan_and_send(self):
        try:
            wifi_data = self.wifi_scanner.scan_wifi()
            self.get_logger().info(f"Wifi data processed -- \n{wifi_data}\n")
            self.send_to_localization_server(wifi_data)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"WiFi scan failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def parse_iw_scan(self, scan_output):
        result = []
        lines = scan_output.splitlines()
        bssid = None
        rssi = None

        for line in lines:
            line = line.strip()
            if line.startswith("BSS "):
                if bssid is not None and rssi is not None:
                    result.append({"bssid": bssid, "rssi": rssi})
                bssid = line.split()[1]
                rssi = None
            elif "signal:" in line:
                try:
                    rssi = float(line.split("signal:")[1].split()[0])
                except (IndexError, ValueError):
                    continue

        if bssid is not None and rssi is not None:
            result.append({"bssid": bssid, "rssi": rssi})

        return result

    def send_to_localization_server(self, wifi_data):
        payload = {
            "device_id": self.device_id,
            "wifi_signals": wifi_data
        }
        headers = {'Content-Type': 'application/json'}

        try:
            response = requests.post(self.api_url, headers=headers, json=payload)
            if response.status_code == 200:
                self.get_logger().info(f"Localization result: {response.json()}")
            else:
                self.get_logger().error(f"Failed to get localization. Status: {response.status_code} | {response.text}")
        except requests.RequestException as e:
            self.get_logger().error(f"HTTP request failed: {e}")

def main(args=None):
    parser = argparse.ArgumentParser(description='Wi-Fi Scanner Node')
    parser.add_argument('--device_id', required=True, help='Device ID (e.g., turtlebot3_LAB)')
    parser.add_argument('--server_ip', required=True, help='Map server IP or hostname (e.g., 192.168.1.100)')
    parser.add_argument('--map_name', required=True, help='Map name to use for localization')
    parser.add_argument('--timer', default=5.0, help='Timer for scan and send (default 10 seconds)')

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
