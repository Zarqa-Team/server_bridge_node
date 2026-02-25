#!/usr/bin/env python3
"""
ROS2 Bridge Node for Competition Simulation Server
Communicates with Flask server at http://127.0.0.1:5000
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
        self.auth_token = None
        
        # Static telemetry data
        self.telemetry_data = {
            "takim_numarasi": 1,
            "iha_enlem": 41.508775,
            "iha_boylam": 36.118335,
            "iha_irtifa": 38,
            "iha_dikilme": 7,
            "iha_yonelme": 210,
            "iha_yatis": -30,
            "iha_hiz": 28,
            "iha_batarya": 50,
            "iha_otonom": 1,
            "iha_kilitlenme": 0,  # 0: not locked
            "hedef_merkez_X": 0,  # Target X 
            "hedef_merkez_Y": 0,  # Target Y 
            "hedef_genislik": 0,  # Target width 
            "hedef_yukseklik": 0,  # Target height
            "gps_saati": {
                "saat": 12,
                "dakika": 10,
                "saniye": 5,
                "milisaniye": 200
            }
        }
        
        # Published Topics
        self.pub_server_time = self.create_publisher(String, '/competition/server_time', 10)
        self.pub_other_uavs = self.create_publisher(String, '/competition/other_uavs', 10)
        self.pub_qr_location = self.create_publisher(String, '/competition/qr_location', 10)
        self.pub_hss_zones = self.create_publisher(String, '/competition/hss_zones', 10)
        self.pub_lockdown_info = self.create_publisher(String, '/competition/lockdown_info', 10)
        self.pub_kamikaze_info = self.create_publisher(String, '/competition/kamikaze_info', 10)
        
        # Subscribed Topics
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
        self.telemetry_timer = self.create_timer(0.6, self.send_telemetry_callback)
        self.qr_hss_timer = self.create_timer(5.0, self.fetch_qr_hss_callback)
          
        self.test_lockdown_timer = self.create_timer(1, self.test_send_lockdown_callback)
        self.test_kamikaze_timer = self.create_timer(1, self.test_send_kamikaze_callback)
        self.get_logger().info("Competition Bridge Node started successfully")
    
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
                self.auth_token = response_data.get("token")
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
    
    def send_telemetry_callback(self):
        try:
            headers = {
                "Content-Type": "application/json",
                "X-Kadi": self.username
            }
            
            response = requests.post(
                f"{self.server_url}/api/telemetri_gonder",
                json=self.telemetry_data,
                headers=headers,
                timeout=self.request_timeout
            )
            
            if response.status_code == 200:
                response_data = response.json()
                
                # Publish server time
                if "sunucusaati" in response_data:
                    msg = String()
                    msg.data = json.dumps(response_data["sunucusaati"])
                    self.pub_server_time.publish(msg)
                
                # Publish other UAVs location 
                if "konumBilgileri" in response_data:
                    msg = String()
                    msg.data = json.dumps(response_data["konumBilgileri"])
                    self.pub_other_uavs.publish(msg)
                
                self.get_logger().debug("Telemetry sent and processed successfully")
            else:
                self.get_logger().warn(
                    f"Telemetry POST returned status {response.status_code}"
                )
        
        except requests.exceptions.Timeout:
            self.get_logger().warn("Telemetry request timed out")
        except requests.exceptions.ConnectionError:
            self.get_logger().warn("Connection error sending telemetry")
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON in telemetry response")
        except Exception as e:
            self.get_logger().error(f"Unexpected error sending telemetry: {str(e)}")
    
    def fetch_qr_hss_callback(self):
        
        # Fetch server time
        self.fetch_server_time()
        
        # Fetch QR coordinates
        self.fetch_qr_coordinates()
        
        # Fetch HSS zones
        self.fetch_hss_zones()
    
    def fetch_qr_coordinates(self):
        try:
            headers = {
                "X-Kadi": self.username
            }
            response = requests.get(
                f"{self.server_url}/api/qr_koordinati",
                headers=headers,
                timeout=self.request_timeout
            )
            
            if response.status_code == 200:
                response_data = response.json()
                msg = String()
                msg.data = json.dumps(response_data)
                self.pub_qr_location.publish(msg)
                self.get_logger().debug("QR coordinates fetched and published")
            else:
                self.get_logger().warn(
                    f"QR coordinates fetch returned status {response.status_code}"
                )
        
        except requests.exceptions.Timeout:
            self.get_logger().warn("QR coordinates request timed out")
        except requests.exceptions.ConnectionError:
            self.get_logger().warn("Connection error fetching QR coordinates")
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON in QR coordinates response")
        except Exception as e:
            self.get_logger().error(f"Unexpected error fetching QR coordinates: {str(e)}")
    
    def fetch_server_time(self):
        """Fetch server time from /api/sunucusaati endpoint."""
        try:
            headers = {
                "X-Kadi": self.username
            }
            response = requests.get(
                f"{self.server_url}/api/sunucusaati",
                headers=headers,
                timeout=self.request_timeout
            )
            
            if response.status_code == 200:
                response_data = response.json()
                msg = String()
                msg.data = json.dumps(response_data)
                self.pub_server_time.publish(msg)
                self.get_logger().debug("Server time fetched and published")
            else:
                self.get_logger().warn(
                    f"Server time fetch returned status {response.status_code}"
                )
        
        except requests.exceptions.Timeout:
            self.get_logger().warn("Server time request timed out")
        except requests.exceptions.ConnectionError:
            self.get_logger().warn("Connection error fetching server time")
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON in server time response")
        except Exception as e:
            self.get_logger().error(f"Unexpected error fetching server time: {str(e)}")
    
    def fetch_hss_zones(self):
        try:
            headers = {
                "X-Kadi": self.username
            }
            response = requests.get(
                f"{self.server_url}/api/hss_koordinatlari",
                headers=headers,
                timeout=self.request_timeout
            )
            
            if response.status_code == 200:
                response_data = response.json()
                msg = String()
                msg.data = json.dumps(response_data)
                self.pub_hss_zones.publish(msg)
                self.get_logger().debug("HSS zones fetched and published")
            else:
                self.get_logger().warn(
                    f"HSS zones fetch returned status {response.status_code}"
                )
        
        except requests.exceptions.Timeout:
            self.get_logger().warn("HSS zones request timed out")
        except requests.exceptions.ConnectionError:
            self.get_logger().warn("Connection error fetching HSS zones")
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON in HSS zones response")
        except Exception as e:
            self.get_logger().error(f"Unexpected error fetching HSS zones: {str(e)}")
    
    def send_lockdown_info(self, end_time_dict, is_autonomous=False):
       
        try:
            headers = {
                "Content-Type": "application/json",
                "X-Kadi": self.username
            }
            
            payload = {
                "kilitlenmeBitisZamani": end_time_dict,
                "otonom_kilitlenme": 1 if is_autonomous else 0
            }
            
            response = requests.post(
                f"{self.server_url}/api/kilitlenme_bilgisi",
                json=payload,
                headers=headers,
                timeout=self.request_timeout
            )
            
            if response.status_code == 200:
                self.get_logger().info("Lockdown info sent successfully")
                # Publish to ROS topic for testing
                msg = String()
                msg.data = json.dumps(payload)
                self.pub_lockdown_info.publish(msg)
            else:
                self.get_logger().warn(
                    f"Lockdown info POST returned status {response.status_code}: {response.text}"
                )
        
        except requests.exceptions.Timeout:
            self.get_logger().error("Lockdown info request timed out")
        except requests.exceptions.ConnectionError:
            self.get_logger().error("Connection error sending lockdown info")
        except Exception as e:
            self.get_logger().error(f"Unexpected error sending lockdown info: {str(e)}")
    
    def send_kamikaze_info(self, start_time_dict, end_time_dict, qr_text):
        
        try:
            headers = {
                "Content-Type": "application/json",
                "X-Kadi": self.username
            }
            
            payload = {
                "kamikazeBaslangicZamani": start_time_dict,
                "kamikazeBitisZamani": end_time_dict,
                "qrMetni": qr_text
            }
            
            response = requests.post(
                f"{self.server_url}/api/kamikaze_bilgisi",
                json=payload,
                headers=headers,
                timeout=self.request_timeout
            )
            
            if response.status_code == 200:
                self.get_logger().info(f"Kamikaze info sent successfully: {qr_text}")
                # Publish to ROS topic for testing/monitoring
                msg = String()
                msg.data = json.dumps(payload)
                self.pub_kamikaze_info.publish(msg)
            else:
                self.get_logger().warn(
                    f"Kamikaze info POST returned status {response.status_code}: {response.text}"
                )
        
        except requests.exceptions.Timeout:
            self.get_logger().error("Kamikaze info request timed out")
        except requests.exceptions.ConnectionError:
            self.get_logger().error("Connection error sending kamikaze info")
        except Exception as e:
            self.get_logger().error(f"Unexpected error sending kamikaze info: {str(e)}")
    
    def update_lockdown_target(self, center_x, center_y, width, height, is_locked=True):
        
        if is_locked:
            self.telemetry_data["iha_kilitlenme"] = 1
            self.telemetry_data["hedef_merkez_X"] = center_x
            self.telemetry_data["hedef_merkez_Y"] = center_y
            self.telemetry_data["hedef_genislik"] = width
            self.telemetry_data["hedef_yukseklik"] = height
            self.get_logger().debug(
                f"Lockdown target updated: X={center_x}, Y={center_y}, W={width}, H={height}"
            )
        else:
            self.telemetry_data["iha_kilitlenme"] = 0
            self.telemetry_data["hedef_merkez_X"] = 0
            self.telemetry_data["hedef_merkez_Y"] = 0
            self.telemetry_data["hedef_genislik"] = 0
            self.telemetry_data["hedef_yukseklik"] = 0
            self.get_logger().debug("Lockdown target cleared")
    
    def kamikaze_subscription_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"Received kamikaze command: {data}")
            
            # Extract data and send to server
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
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"Received lockdown command: {data}")
            
            # Extract data and send to server
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
    
    def test_send_lockdown_callback(self):
       
        dummy_end_time = {
            "saat": 12,
            "dakika": 10,
            "saniye": 15,
            "milisaniye": 500
        }
        self.get_logger().info("Sending dummy lockdown info")
        self.send_lockdown_info(dummy_end_time, is_autonomous=True)
    
    def test_send_kamikaze_callback(self):
        
        dummy_start_time = {
            "saat": 12,
            "dakika": 10,
            "saniye": 0,
            "milisaniye": 0
        }
        dummy_end_time = {
            "saat": 12,
            "dakika": 10,
            "saniye": 25,
            "milisaniye": 250
        }
        self.get_logger().info("Sending dummy kamikaze info")
        self.send_kamikaze_info(dummy_start_time, dummy_end_time, "DUMMY_QR_CODE_123")

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
