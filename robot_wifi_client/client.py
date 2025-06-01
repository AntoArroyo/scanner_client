import rclpy
from rclpy.node import Node
import subprocess
import json
import requests

class WifiScannerNode(Node):
    def __init__(self):
        super().__init__('wifi_scanner_node')
        self.timer = self.create_timer(30.0, self.scan_and_send)  # every 30s
        self.map_name = 'LAB'  
        self.device_id = 'turtlebot3'  
        self.api_url = f'http://map_server:8000/localize_basic_graph/{self.map_name}'

    def scan_and_send(self):
        try:
            scan_output = subprocess.check_output(
                ['iw', 'dev', 'wlan0', 'scan'], stderr=subprocess.DEVNULL
            ).decode()
            wifi_data = self.parse_iw_scan(scan_output)
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

        # Add the last one
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
    rclpy.init(args=args)
    node = WifiScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
