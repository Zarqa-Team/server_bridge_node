#!/usr/bin/env python3
"""
ROS2 Bridge Node for Competition Simulation Server
Communicates with Flask server at http://127.0.0.1:5000

Subscribes to aggregated telemetry data from TelemetryAggregatorNode
and sends it to the server.
"""

import json
import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CompetitionBridgeNode(Node):

    def __init__(self):
        super().__init__('competition_bridge')
        
        # Configuration
        self.server_url = "http://127.0.0.1:5000"
        self.username = "zarqa"
        self.password = "1234"
        self.request_timeout = 5.0
        
        # Cache for telemetry data received from aggregator
        self.telemetry_data = {}
        
        # Published Topics for server responses
        self.pub_server_time = self.create_publisher(String, '/competition/server_time', 10)
        self.pub_other_uavs = self.create_publisher(String, '/competition/other_uavs', 10)
        self.pub_qr_location = self.create_publisher(String, '/competition/qr_location', 10)
        self.pub_hss_zones = self.create_publisher(String, '/competition/hss_zones', 10)
        self.pub_lockdown_info = self.create_publisher(String, '/competition/lockdown_info', 10)
        self.pub_kamikaze_info = self.create_publisher(String, '/competition/kamikaze_info', 10)
        
        # Subscribe to telemetry data from aggregator node
        self.sub_telemetry = self.create_subscription(
            String,
            '/competition/telemetry_data',
            self.telemetry_callback,
            10
        )
        
        # Subscriptions for mission control commands
        self.sub_kamikaze = self.create_subscription(
            String,
            '/mission/kamikaze_command',
            self.kamikaze_subscription_callback,
            10
        )
        
        self.sub_lockdown = self.create_subscription(
            String,
            '/mission/lockdown_command',
            self.lockdown_subscription_callback,
            10
        )
        
        self.get_logger().info("Initializing Competition Bridge Node")
        
        # Login at startup
        self.login()
        
        # Creating timers
        # Send telemetry to server at 0.6 second intervals
        self.telemetry_timer = self.create_timer(0.6, self.send_telemetry_callback)
        # Fetch QR and HSS data from server every 5 seconds
        self.qr_hss_timer = self.create_timer(5.0, self.fetch_qr_hss_callback)
        
        self.get_logger().info("Competition Bridge Node started successfully")
        self.get_logger().info(f"Telemetry server: {self.server_url}")
        self.get_logger().info("Subscribing to telemetry data from aggregator node")
    
    def telemetry_callback(self, msg):
        """Receive aggregated telemetry data from TelemetryAggregatorNode"""
        try:
            self.telemetry_data = json.loads(msg.data)
            self.get_logger().debug("Telemetry data received from aggregator")
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse telemetry data from aggregator")
    
    def login(self):
        try:
            login_payload = {
                "kadi": self.username,
                "sifre": self.password
            }
            
            response = requests.post(
                f"{self.server_url}/api/giris",
                json=login_payload,
                timeout=self.request_timeout
            )
            
            if response.status_code == 200:
                response_data = response.json()
                self.get_logger().info(f"Login successful: {response_data}")
            else:
                self.get_logger().error(
                    f"Login failed with status {response.status_code}: {response.text}"
                )
        except requests.exceptions.Timeout:
            self.get_logger().error("Login request timed out")
        except requests.exceptions.ConnectionError:
            self.get_logger().error("Connection error during login")
        except Exception as e:
            self.get_logger().error(f"Unexpected error during login: {str(e)}")
    
    def _make_request(self, method, endpoint, payload=None, success_handler=None):
        """Generic HTTP request handler with error handling.
        
        Args:
            method: 'GET' or 'POST'
            endpoint: API endpoint path (e.g., 'telemetri_gonder')
            payload: JSON payload for POST requests
            success_handler: Callback function called with response_data on success
        """
        try:
            headers = {"X-Kadi": self.username}
            if method.upper() == "POST":
                headers["Content-Type"] = "application/json"
            
            url = f"{self.server_url}/api/{endpoint}"
            
            if method.upper() == "GET":
                response = requests.get(url, headers=headers, timeout=self.request_timeout)
            else:
                response = requests.post(url, json=payload, headers=headers, timeout=self.request_timeout)
            
            if response.status_code == 200:
                response_data = response.json()
                if success_handler:
                    success_handler(response_data)
                return response_data
            else:
                self.get_logger().warn(f"{endpoint} returned status {response.status_code}")
                return None
        
        except requests.exceptions.Timeout:
            self.get_logger().warn(f"{endpoint} timed out")
        except requests.exceptions.ConnectionError:
            self.get_logger().warn(f"Connection error: {endpoint}")
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON in {endpoint} response")
        except Exception as e:
            self.get_logger().error(f"Error in {endpoint}: {str(e)}")
        
        return None
    
    def send_telemetry_callback(self):
        """Send aggregated telemetry data to server"""
        if not self.telemetry_data:
            return
        
        def handle_telemetry_response(response_data):
            # Publish server time if available
            if "sunucusaati" in response_data:
                msg = String()
                msg.data = json.dumps(response_data["sunucusaati"])
                self.pub_server_time.publish(msg)
            
            # Publish other UAVs location if available
            if "konumBilgileri" in response_data:
                msg = String()
                msg.data = json.dumps(response_data["konumBilgileri"])
                self.pub_other_uavs.publish(msg)
            
            self.get_logger().info("Telemetry sent successfully")
        
        self._make_request("POST", "telemetri_gonder", self.telemetry_data, handle_telemetry_response)
    
    def fetch_qr_hss_callback(self):
        """Fetch QR, HSS, and server time data from server"""
        self.fetch_server_time()
        self.fetch_qr_coordinates()
        self.fetch_hss_zones()
    
    def fetch_qr_coordinates(self):
        """Fetch QR coordinates from server"""
        def handle_qr_response(response_data):
            msg = String()
            msg.data = json.dumps(response_data)
            self.pub_qr_location.publish(msg)
            self.get_logger().info("QR coordinates fetched and published")
        
        self._make_request("GET", "qr_koordinati", success_handler=handle_qr_response)
    
    def fetch_server_time(self):
        """Fetch server time from server"""
        def handle_server_time_response(response_data):
            msg = String()
            msg.data = json.dumps(response_data)
            self.pub_server_time.publish(msg)
            self.get_logger().info("Server time fetched and published")
        
        self._make_request("GET", "sunucusaati", success_handler=handle_server_time_response)
    
    def fetch_hss_zones(self):
        """Fetch HSS zones from server"""
        def handle_hss_response(response_data):
            msg = String()
            msg.data = json.dumps(response_data)
            self.pub_hss_zones.publish(msg)
            self.get_logger().info("HSS zones fetched and published")
        
        self._make_request("GET", "hss_koordinatlari", success_handler=handle_hss_response)
    
    def send_lockdown_info(self, end_time_dict, is_autonomous=False):
        """Send lockdown info to server"""
        payload = {
            "kilitlenmeBitisZamani": end_time_dict,
            "otonom_kilitlenme": 1 if is_autonomous else 0
        }
        
        def handle_lockdown_response(response_data):
            self.get_logger().info("Lockdown info sent successfully")
            msg = String()
            msg.data = json.dumps(payload)
            self.pub_lockdown_info.publish(msg)
        
        self._make_request("POST", "kilitlenme_bilgisi", payload, handle_lockdown_response)
    
    def send_kamikaze_info(self, start_time_dict, end_time_dict, qr_text):
        """Send kamikaze info to server"""
        payload = {
            "kamikazeBaslangicZamani": start_time_dict,
            "kamikazeBitisZamani": end_time_dict,
            "qrMetni": qr_text
        }
        
        def handle_kamikaze_response(response_data):
            self.get_logger().info(f"Kamikaze info sent successfully: {qr_text}")
            msg = String()
            msg.data = json.dumps(payload)
            self.pub_kamikaze_info.publish(msg)
        
        self._make_request("POST", "kamikaze_bilgisi", payload, handle_kamikaze_response)
    
    def update_lockdown_target(self, center_x, center_y, width, height, is_locked=True):
        """Update target lock status and coordinates in telemetry data"""
        if is_locked:
            self.telemetry_data["iha_kilitlenme"] = 1
            self.telemetry_data["hedef_merkez_X"] = center_x
            self.telemetry_data["hedef_merkez_Y"] = center_y
            self.telemetry_data["hedef_genislik"] = width
            self.telemetry_data["hedef_yukseklik"] = height
            self.get_logger().debug(
                f"Lockdown target updated: center=({center_x}, {center_y}), size={width}x{height}"
            )
        else:
            self.telemetry_data["iha_kilitlenme"] = 0
            self.telemetry_data["hedef_merkez_X"] = 0
            self.telemetry_data["hedef_merkez_Y"] = 0
            self.telemetry_data["hedef_genislik"] = 0
            self.telemetry_data["hedef_yukseklik"] = 0
            self.get_logger().debug("Lockdown target cleared")
    
    def kamikaze_subscription_callback(self, msg):
        """Handle kamikaze command from mission control"""
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"Received kamikaze command: {data}")
            
            start_time = data.get("start_time", {})
            end_time = data.get("end_time", {})
            qr_text = data.get("qr_text", "")
            
            if start_time and end_time and qr_text:
                self.send_kamikaze_info(start_time, end_time, qr_text)
            else:
                self.get_logger().warn("Invalid kamikaze command format")
        
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse kamikaze command JSON")
        except Exception as e:
            self.get_logger().error(f"Error processing kamikaze command: {str(e)}")
    
    def lockdown_subscription_callback(self, msg):
        """Handle lockdown command from mission control"""
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"Received lockdown command: {data}")
            
            end_time = data.get("end_time", {})
            is_autonomous = data.get("is_autonomous", False)
            
            # Optional: Update target coordinates if provided
            if "center_x" in data and "center_y" in data:
                self.update_lockdown_target(
                    data["center_x"],
                    data["center_y"],
                    data.get("width", 0),
                    data.get("height", 0),
                    is_locked=True
                )
            
            if end_time:
                self.send_lockdown_info(end_time, is_autonomous=is_autonomous)
            else:
                self.get_logger().warn("Invalid lockdown command format")
        
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse lockdown command JSON")
        except Exception as e:
            self.get_logger().error(f"Error processing lockdown command: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = CompetitionBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
