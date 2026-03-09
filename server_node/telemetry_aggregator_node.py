#!/usr/bin/env python3
"""
Telemetry Aggregator Node
Subscribes to PX4 SITL topics and publishes aggregated telemetry data
"""

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleGlobalPosition, BatteryStatus, VehicleStatus, AirspeedValidated


class TelemetryAggregatorNode(Node):

    def __init__(self):
        super().__init__('telemetry_aggregator')
        
        # QoS profile for PX4 SITL topics
        self.px4_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize telemetry data with zeros
        self.telemetry_data = {
            "takim_numarasi": 1,
            "iha_enlem": 0.0,
            "iha_boylam": 0.0,
            "iha_irtifa": 0.0,
            "iha_dikilme": 0.0,  # roll
            "iha_yonelme": 0.0,  # yaw
            "iha_yatis": 0.0,    # pitch
            "iha_hiz": 0.0,      # velocity
            "iha_batarya": 0.0,
            "iha_otonom": 0,
            "iha_kilitlenme": 0,  # 0: not locked
            "hedef_merkez_X": 0,  # Target X 
            "hedef_merkez_Y": 0,  # Target Y 
            "hedef_genislik": 0,  # Target width 
            "hedef_yukseklik": 0,  # Target height
            "gps_saati": {
                "saat": 0,
                "dakika": 0,
                "saniye": 0,
                "milisaniye": 0
            }
        }
        
        # Track velocity sources for fallback logic
        self.last_local_velocity = 0.0  # Ground speed from local position
        self.last_airspeed = 0.0        # True airspeed (more reliable in SITL)
        
        # Publisher
        self.pub_telemetry_data = self.create_publisher(String, '/competition/telemetry_data', 10)
        
        # Subscriptions to PX4 SITL topics
        self.sub_attitude = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            self.px4_qos_profile
        )
        
        self.sub_local_position = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.local_position_callback,
            self.px4_qos_profile
        )
        
        self.sub_global_position = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.global_position_callback,
            self.px4_qos_profile
        )
        
        self.sub_battery = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status_v1',
            self.battery_callback,
            self.px4_qos_profile
        )
        
        self.sub_vehicle_status = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            self.px4_qos_profile
        )
        
        self.sub_airspeed = self.create_subscription(
            AirspeedValidated,
            '/fmu/out/airspeed_validated_v1',
            self.airspeed_callback,
            self.px4_qos_profile
        )
        
        # Publish telemetry at 1 Hz
        self.telemetry_publisher_timer = self.create_timer(1.0, self.publish_telemetry_callback)
        
        self.get_logger().info("Telemetry Aggregator Node started successfully")
    
    def quaternion_to_euler(self, q):
        """Convert quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw) in degrees."""
        w, x, y, z = q[3], q[0], q[1], q[2]
        
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    def gps_time_to_dict(self, gps_time_us):
        """Convert GPS time in microseconds to dictionary with saat, dakika, saniye, milisaniye."""
        try:
            total_seconds = gps_time_us / 1_000_000
            hours = int((total_seconds // 3600) % 24)
            minutes = int((total_seconds // 60) % 60)
            seconds = int(total_seconds % 60)
            milliseconds = int((gps_time_us % 1_000_000) / 1000)
            
            return {
                "saat": max(0, min(23, hours)),
                "dakika": max(0, min(59, minutes)),
                "saniye": max(0, min(59, seconds)),
                "milisaniye": max(0, min(999, milliseconds))
            }
        except Exception as e:
            self.get_logger().warn(f"Error converting GPS time: {str(e)}")
            return {"saat": 0, "dakika": 0, "saniye": 0, "milisaniye": 0}
    
    def attitude_callback(self, msg):
        """Update telemetry with vehicle attitude"""
        try:
            roll, pitch, yaw = self.quaternion_to_euler(msg.q)
            
            roll = max(-90, min(90, roll))
            pitch = max(-90, min(90, pitch))
            yaw = yaw % 360
            if yaw < 0:
                yaw += 360
            
            self.telemetry_data["iha_dikilme"] = round(roll, 2)
            self.telemetry_data["iha_yonelme"] = round(yaw, 2)
            self.telemetry_data["iha_yatis"] = round(pitch, 2)
            
            self.get_logger().debug(f"Attitude: roll={roll:.2f}°, yaw={yaw:.2f}°, pitch={pitch:.2f}°")
        except Exception as e:
            self.get_logger().error(f"Error processing attitude data: {str(e)}")
    
    def local_position_callback(self, msg):
        """Update telemetry with local velocity magnitude"""
        try:
            velocity = math.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2)
            velocity = max(0, velocity)
            self.last_local_velocity = velocity
            
            if velocity > 0.5:
                self.telemetry_data["iha_hiz"] = round(velocity, 2)
            elif self.last_airspeed > 0:
                self.telemetry_data["iha_hiz"] = round(self.last_airspeed, 2)
            else:
                self.telemetry_data["iha_hiz"] = round(velocity, 2)
        except Exception as e:
            self.get_logger().error(f"Error processing local position data: {str(e)}")
    
    def global_position_callback(self, msg):
        """Update telemetry with global position"""
        try:
            if abs(msg.lat) > 1000 or abs(msg.lon) > 1000:
                latitude = msg.lat / 1e7
                longitude = msg.lon / 1e7
            else:
                latitude = float(msg.lat)
                longitude = float(msg.lon)
            
            altitude = msg.alt
            
            latitude = max(-90, min(90, latitude))
            longitude = max(-180, min(180, longitude))
            altitude = max(0, altitude)
            
            self.telemetry_data["iha_enlem"] = round(latitude, 6)
            self.telemetry_data["iha_boylam"] = round(longitude, 6)
            self.telemetry_data["iha_irtifa"] = round(altitude, 2)
            self.telemetry_data["gps_saati"] = self.gps_time_to_dict(msg.timestamp)
        except Exception as e:
            self.get_logger().error(f"Error processing global position data: {str(e)}")
    
    def battery_callback(self, msg):
        """Update telemetry with battery status"""
        try:
            battery_percent = msg.remaining if hasattr(msg, 'remaining') else 0.0
            battery_percent = max(0.0, min(100.0, battery_percent)) * 100
            self.telemetry_data["iha_batarya"] = round(battery_percent, 2)
        except Exception as e:
            self.get_logger().error(f"Error processing battery data: {str(e)}")
    
    def vehicle_status_callback(self, msg):
        """Update telemetry with vehicle status"""
        try:
            is_autonomous = msg.nav_state in [3, 14]
            self.telemetry_data["iha_otonom"] = 1 if is_autonomous else 0
        except Exception as e:
            self.get_logger().error(f"Error processing vehicle status data: {str(e)}")
    
    def airspeed_callback(self, msg):
        """Update telemetry with airspeed"""
        try:
            airspeed = msg.true_airspeed_m_s if hasattr(msg, 'true_airspeed_m_s') else 0.0
            self.last_airspeed = max(0.0, airspeed)
        except Exception as e:
            self.get_logger().error(f"Error processing airspeed data: {str(e)}")
    
    def publish_telemetry_callback(self):
        """Publish aggregated telemetry data as JSON"""
        try:
            msg = String()
            msg.data = json.dumps(self.telemetry_data)
            self.pub_telemetry_data.publish(msg)
            self.get_logger().debug("Telemetry published")
        except Exception as e:
            self.get_logger().error(f"Error publishing telemetry: {str(e)}")
    
    def update_lockdown_target(self, center_x, center_y, width, height, is_locked=True):
        """Update target lock status and coordinates"""
        if is_locked:
            self.telemetry_data["iha_kilitlenme"] = 1
            self.telemetry_data["hedef_merkez_X"] = center_x
            self.telemetry_data["hedef_merkez_Y"] = center_y
            self.telemetry_data["hedef_genislik"] = width
            self.telemetry_data["hedef_yukseklik"] = height
        else:
            self.telemetry_data["iha_kilitlenme"] = 0
            self.telemetry_data["hedef_merkez_X"] = 0
            self.telemetry_data["hedef_merkez_Y"] = 0
            self.telemetry_data["hedef_genislik"] = 0
            self.telemetry_data["hedef_yukseklik"] = 0


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryAggregatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
