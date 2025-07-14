import subprocess
import re
import shutil
import os

class WiFiScanner:
    """
    Class responsible for WiFi scanning operations.
    Supports both nmcli and iw scanning methods.
    """
    
    def __init__(self, logger, wifi_interface=None, wifi_top_n=10):
        self.logger = logger
        self.wifi_interface = wifi_interface or self._detect_wifi_interface()
        self.wifi_top_n = wifi_top_n # n APs limit to process
        self.sudo_available = self._check_sudo_available()
        
        if not self.sudo_available:
            self.logger.warn("sudo is not available. Some Wi-Fi scanning features may be limited.")
            
        self.logger.info(f"WiFi Scanner initialized with interface: {self.wifi_interface}")
    
    def _check_sudo_available(self):
        """Check if sudo is available on the system."""
        try:
            sudo_path = shutil.which('sudo')
            return sudo_path is not None
        except Exception as e:
            self.logger.error(f"Error checking for sudo: {str(e)}")
            return False
    
    def _run_with_sudo(self, cmd_list):
        """Run a command with sudo if available."""
        try:
            if self.sudo_available:
                cmd = ['sudo'] + cmd_list
                return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            else:
                # Try to run without sudo, but this may fail for commands requiring privileges
                self.logger.warn(f"Running command without sudo: {cmd_list}")
                return subprocess.run(cmd_list, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except Exception as e:
            self.logger.error(f"Error running command {cmd_list}: {str(e)}")
            return None
    
    def _detect_wifi_interface(self):
        """
        Automatically detect the Wi-Fi interface.
        Returns the name of the first active wireless interface or a fallback value.
        """
        WIFI_INTERFACE_FALLBACK = 'wlo1'
        WIFI_INTERFACE_RPI_FALLBACK = 'wlan0'
        
        try:
            # Try using ip command first (most modern Linux systems)
            result = subprocess.run(['ip', 'link', 'show'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if result.returncode == 0:
                output = result.stdout.decode('utf-8')
                # Look for wireless interfaces (wlan, wlp, wlo, etc.)
                wifi_interfaces = re.findall(r'\d+: (wl\w+):', output)
                if wifi_interfaces:
                    # Check if the interface is up
                    for interface in wifi_interfaces:
                        # Check if interface is up
                        up_check = subprocess.run(
                            ['ip', 'link', 'show', interface], 
                            stdout=subprocess.PIPE, 
                            stderr=subprocess.PIPE
                        )
                        if up_check.returncode == 0 and 'state UP' in up_check.stdout.decode('utf-8'):
                            self.logger.info(f"Found active Wi-Fi interface: {interface}")
                            return interface
            
            # Try using iwconfig as a fallback
            result = subprocess.run(['iwconfig'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if result.returncode == 0:
                output = result.stdout.decode('utf-8')
                lines = output.split('\n')
                for line in lines:
                    if 'IEEE 802.11' in line:  # This indicates a wireless interface
                        interface = line.split()[0]
                        self.logger.info(f"Found Wi-Fi interface using iwconfig: {interface}")
                        return interface
            
            # Try using nmcli as another fallback
            if shutil.which('nmcli'):
                result = subprocess.run(['nmcli', 'device', 'status'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                if result.returncode == 0:
                    output = result.stdout.decode('utf-8')
                    lines = output.split('\n')
                    for line in lines:
                        if 'wifi' in line and 'connected' in line:
                            parts = line.split()
                            if parts:
                                self.logger.info(f"Found Wi-Fi interface using nmcli: {parts[0]}")
                                return parts[0]
            
            # If we're on a Raspberry Pi, use the typical interface name
            if os.path.exists('/proc/device-tree/model'):
                with open('/proc/device-tree/model', 'r') as f:
                    model = f.read()
                    if 'Raspberry Pi' in model:
                        self.logger.info(f"Detected Raspberry Pi, using interface: {WIFI_INTERFACE_RPI_FALLBACK}")
                        return WIFI_INTERFACE_RPI_FALLBACK
                        
            # Fallback to default
            self.logger.warn(f"Could not detect Wi-Fi interface, using fallback: {WIFI_INTERFACE_FALLBACK}")
            return WIFI_INTERFACE_FALLBACK
            
        except Exception as e:
            self.logger.error(f"Error detecting Wi-Fi interface: {str(e)}")
            self.logger.warn(f"Using fallback Wi-Fi interface: {WIFI_INTERFACE_FALLBACK}")
            return WIFI_INTERFACE_FALLBACK
    
    def scan_wifi(self, method='iw'):
        """
        Scan for WiFi networks using the specified method.
        
        Args:
            method (str): Scanning method, either 'nmcli' or 'iw'
            
        Returns:
            list: List of WiFi networks with BSSID as dictionary keys
        """
        if method == 'nmcli':
            return self.scan_wifi_nmcli()
        elif method == 'iw':
            return self.scan_wifi_iw()
        else:
            self.logger.error(f"Unknown Wi-Fi scan method: {method}")
            return []
    
    def scan_wifi_nmcli(self):
        """
        Scan for WiFi networks using nmcli command.
        
        Returns:
            list: List of WiFi networks with BSSID as dictionary keys
        """
        try:
            self.logger.info("Starting Wi-Fi scan with nmcli...")
            # Run nmcli rescan with sudo
            rescan_result = self._run_with_sudo(['nmcli', 'dev', 'wifi', 'rescan'])
            if rescan_result and rescan_result.returncode != 0:
                self.logger.warn(f"nmcli rescan failed: {rescan_result.stderr.decode()}")
            
            # Run nmcli dev wifi without sudo as it only reads information
            result = subprocess.run(['nmcli', '-t', '-f', 'SIGNAL,BSSID,SSID', 'dev', 'wifi'], 
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            if result.returncode != 0:
                self.logger.error(f"nmcli dev wifi failed: {result.stderr.decode()}")
                return []
                
            wifi_info = result.stdout.decode().strip()
            wifi_list = self.parse_nmcli_wifi_info(wifi_info)
            self.logger.info("Wi-Fi scan with nmcli completed")
            return wifi_list
        except Exception as e:
            self.logger.error(f"Error scanning Wi-Fi with nmcli: {str(e)}")
            return []
    
    def parse_nmcli_wifi_info(self, wifi_info):
        """
        Parse the output from nmcli command.
        
        Args:
            wifi_info (str): Output from nmcli command
            
        Returns:
            list: List of WiFi networks with BSSID as dictionary keys
        """
        wifi_dict = {}  # Use a dictionary instead of a list to avoid duplicates
        for line in wifi_info.split('\n'):
            if line:
                clean_line = line.replace("\\", "")
                signal = int(clean_line[:2])
                bssid = clean_line[3:20]
                ssid = line[26:].strip() if len(line) > 26 else ""
                # Use BSSID as the key
                wifi_dict[bssid] = {'SSID': ssid, 'BSSID': bssid, 'SIGNAL': signal}
        
        # Convert to list for sorting
        return sorted(wifi_dict.values(), key=lambda x: x['SIGNAL'], reverse=True)[:self.wifi_top_n]
    
    def scan_wifi_iw(self):
        """
        Scan for WiFi networks using iw command.
        
        Returns:
            list: List of WiFi networks with BSSID as dictionary keys
        """
        try:
            self.logger.info("Starting Wi-Fi scan with iw...")
            # Run iw scan with sudo
            result = self._run_with_sudo(['iw', 'dev', self.wifi_interface, 'scan'])
            
            if not result:
                self.logger.error("Failed to run iw command")
                return []
                
            if result.returncode != 0:
                self.logger.error(f"iw scan failed: {result.stderr.decode()}")
                return []
                
            wifi_info = result.stdout.decode().strip()
            wifi_list = self.parse_iw_wifi_info(wifi_info)
            self.logger.info("Wi-Fi scan with iw completed")
            return wifi_list
        except Exception as e:
            self.logger.error(f"Error scanning Wi-Fi with iw: {str(e)}")
            return []
    
    def parse_iw_wifi_info(self, wifi_info):
        """
        Parse the output from iw command.
        
        Args:
            wifi_info (str): Output from iw command
            
        Returns:
            list: List of WiFi networks with BSSID as dictionary keys
        """
        wifi_dict = {}  # Use a dictionary instead of a list to avoid duplicates
        current_network = {}
        for line in wifi_info.split('\n'):
            line = line.strip()
            if line.startswith('BSS'):
                if current_network and 'bssid' in current_network:
                    # Add network to dictionary using BSSID as key
                    self.add_network_to_dict(wifi_dict, current_network)
                bssid = line.split()[1].split('(')[0]
                current_network = {'bssid': bssid} if bssid != 'Load:' else {}
            elif line.startswith('SSID:'):
                current_network['SSID'] = line.split(':', 1)[1].strip() if len(line.split(':', 1)) > 1 else ""
            elif line.startswith('signal:'):
                current_network['rssi'] = float(line.split()[1].split()[0])
        
        if current_network and 'bssid' in current_network:
            self.add_network_to_dict(wifi_dict, current_network)
        
        # Convert to list for sorting
        wifi_list = list(wifi_dict.values())
        return sorted(wifi_list, key=lambda x: x['rssi'], reverse=True)[:self.wifi_top_n]
    
    def add_network_to_dict(self, wifi_dict, network):
        """
        Add a network to the WiFi dictionary, using BSSID as key.
        
        Args:
            wifi_dict (dict): Dictionary of WiFi networks
            network (dict): Network to add
        """
        # Ensure all required keys exist
        if 'rssi' not in network:
            self.logger.warn(f"Missing SIGNAL for network: {network}")
            network['rssi'] = float('-inf')
        if 'SSID' not in network:
            self.logger.warn(f"Missing SSID for network: {network}")
            network['SSID'] = ''
        if 'bssid' not in network:
            self.logger.warn(f"Missing BSSID for network: {network}")
            network['bssid'] = ''
            return  # Skip networks without BSSID as we need it for the key
        
        # If this BSSID already exists in the dictionary, only update if the new signal is stronger
        bssid = network['bssid']
        if bssid in wifi_dict:
            if network['rssi'] > wifi_dict[bssid]['rssi']:
                wifi_dict[bssid] = network
        else:
            wifi_dict[bssid] = network
