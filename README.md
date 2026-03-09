# Server Node - ROS2 Bridge for Competition Simulation

A modular ROS2 package that connects a drone (UAV) simulator to a Flask-based competition server. This system captures drone sensor data, processes it, and communicates with a remote server.

## 📋 Table of Contents
- [Overview](#overview)
- [Architecture](#architecture)
- [What is ROS?](#what-is-ros)
- [Installation](#installation)
- [Running the System](#running-the-system)
- [Understanding the Nodes](#understanding-the-nodes)
- [Data Flow](#data-flow)
- [Topics Reference](#topics-reference)
- [Examples](#examples)
- [Troubleshooting](#troubleshooting)

---

## Overview

This package consists of **two separate nodes** that work together:

1. **Telemetry Aggregator Node** - Reads drone sensor data and publishes it
2. **Competition Bridge Node** - Receives sensor data and sends it to the server

### What This Does

```
PX4 Drone Simulator
       ↓
   [Sensors: GPS, Compass, Battery, etc.]
       ↓
Telemetry Aggregator Node (processes raw data)
       ↓
   [/competition/telemetry_data]
       ↓
Competition Bridge Node (sends to server)
       ↓
   Flask Server (http://127.0.0.1:5000)
```

---

## 🏗️ Architecture

### Why Two Nodes?

**Separation of Concerns**: Each node has one job
- **Aggregator**: Reads sensors, calculates values
- **Bridge**: Handles server communication

**Benefits**:
- Easy to test each part independently
- Reusable components (other nodes can subscribe to telemetry)
- Cleaner code and fewer bugs
- Faster development and debugging

### Node Diagram

```
┌─────────────────────────────────────────────────────┐
│  Telemetry Aggregator Node                          │
│  ────────────────────────────────────────────────    │
│  Input: PX4 SITL Topics                             │
│  • /fmu/out/vehicle_attitude (drone orientation)    │
│  • /fmu/out/vehicle_local_position (location)       │
│  • /fmu/out/vehicle_global_position (GPS)           │
│  • /fmu/out/battery_status (battery level)          │
│  • /fmu/out/vehicle_status (is autonomous?)         │
│  • /fmu/out/airspeed (true airspeed)                │
│                                                     │
│  Processing:                                        │
│  ✓ Converts quaternion → Euler angles               │
│  ✓ Validates coordinate formats (SITL vs Real)      │
│  ✓ Selects best velocity source (ground/air)        │
│  ✓ Converts time formats                            │
│                                                     │
│  Output: /competition/telemetry_data (1 Hz)         │
└─────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────┐
│  Competition Bridge Node                            │
│  ────────────────────────────────────────────────    │
│  Input:                                             │
│  • /competition/telemetry_data (from aggregator)    │
│  • /mission/lockdown_command (from mission control) │
│  • /mission/kamikaze_command (from mission control) │
│                                                     │
│  Processing:                                        │
│  ✓ HTTP login to Flask server                       │
│  ✓ Sends telemetry data (0.6s interval)             │
│  ✓ Fetches QR, HSS, server time (5s interval)       │
│  ✓ Handles mission commands                         │
│                                                     │
│  Output: Published topics + HTTP requests           │
│  • /competition/server_time                         │
│  • /competition/other_uavs                          │
│  • /competition/qr_location                         │
│  • /competition/hss_zones                           │
│  • /competition/lockdown_info                       │
│  • /competition/kamikaze_info                       │
└─────────────────────────────────────────────────────┘
```

---

## 📚 What is ROS?

**ROS (Robot Operating System)** is middleware that allows different programs to communicate.

### Key Concepts

| Concept | Explanation | Example |
|---------|-------------|---------|
| **Node** | Standalone Python/C++ program | `telemetry_aggregator`, `competition_bridge` |
| **Topic** | Named communication channel | `/competition/telemetry_data` |
| **Publish** | Send data to a topic | Aggregator publishes telemetry |
| **Subscribe** | Receive data from a topic | Bridge subscribes to telemetry |
| **Message** | Data structure | `std_msgs/String` (text data) |

### Real-World Analogy

```
ROS is like a newsletter system:
- Nodes = Authors and Readers
- Topics = Newsletters  
- Publishing = Author writes newsletter
- Subscribing = Reader subscribes to newsletter
- Messages = Content of the newsletter
```

---

## 💾 Installation

### Prerequisites

```bash
# Check you have these:
python3 --version      # Should be 3.10+
ros2 --version         # Should be Humble
```

### Install Dependencies

```bash
# Navigate to workspace
cd ~/ros2_ws

# Install system dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install Python packages
pip install requests
pip install px4-msgs  # If not already installed

# Install px4_msgs package (for message types)
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ..
colcon build --packages-select px4_msgs
source install/setup.bash
```

### Build Package

```bash
cd ~/ros2_ws
colcon build --packages-select server_node
source install/setup.bash  # Enable the package
```

---

## 🚀 Running the System

### Quick Start (Recommended: 3 Terminals)

**Terminal 1: Setup environment**
```bash
cd ~/ros2_ws
source install/setup.bash
```

**Terminal 2: Start Telemetry Aggregator**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run server_node telemetry_aggregator
```

**Terminal 3: Start Competition Bridge**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run server_node bridge
```

### Verify It's Working

In a fourth terminal:
```bash
source ~/ros2_ws/install/setup.bash

# List all active topics
ros2 topic list

# Watch telemetry being published
ros2 topic echo /competition/telemetry_data

# Check bridge status
ros2 topic echo /competition/server_time
```

---

## 🔍 Understanding the Nodes

### Node 1: Telemetry Aggregator

**File**: `server_node/telemetry_aggregator_node.py`  
**Purpose**: Read drone sensors, process data, publish results  
**Launch**: `ros2 run server_node telemetry_aggregator`

#### What It Reads (Input Topics)

PX4 sensor data on these topics:

| Topic | Contains | Example |
|-------|----------|---------|
| `/fmu/out/vehicle_attitude` | Drone orientation (quaternion) | `[x, y, z, w]` |
| `/fmu/out/vehicle_local_position_v1` | Local position & velocity | Position in meters |
| `/fmu/out/vehicle_global_position` | GPS coordinates | Latitude, longitude, altitude |
| `/fmu/out/battery_status_v1` | Battery info | Battery percentage |
| `/fmu/out/vehicle_status` | Drone state | Is it autonomous? Armed? |
| `/fmu/out/airspeed_validated_v1` | Wind-relative speed | Airspeed in m/s |

#### What It Does

1. **Quaternion → Euler Angles**: Converts orientation from quaternion format to human-readable degrees (roll, pitch, yaw)
2. **Velocity Selection**: Chooses best velocity (ground speed or airspeed) based on reliability
3. **Format Validation**: Handles different SITL vs real hardware formats for GPS
4. **Time Conversion**: Converts microsecond timestamps to human-readable time

#### What It Publishes (Output)

**Topic**: `/competition/telemetry_data`  
**Frequency**: 1 Hz (once per second)  
**Format**: JSON string containing all processed telemetry

**Example output** (auto-published every 1 second):
```json
{
  "takim_numarasi": 1,
  "iha_enlem": 41.508775,
  "iha_boylam": 36.118335,
  "iha_irtifa": 38.5,
  "iha_dikilme": 7.2,
  "iha_yonelme": 210.5,
  "iha_yatis": -30.1,
  "iha_hiz": 28.4,
  "iha_batarya": 85.5,
  "iha_otonom": 1,
  "iha_kilitlenme": 0,
  "hedef_merkez_X": 0,
  "hedef_merkez_Y": 0,
  "hedef_genislik": 0,
  "hedef_yukseklik": 0,
  "gps_saati": {
    "saat": 12,
    "dakika": 10,
    "saniye": 5,
    "milisaniye": 200
  }
}
```

---

### Node 2: Competition Bridge

**File**: `server_node/bridge.py`  
**Purpose**: Communicate with Flask server, handle mission commands  
**Launch**: `ros2 run server_node bridge`

#### What It Reads (Input Topics)

| Topic | Publishes | When |
|-------|-----------|------|
| `/competition/telemetry_data` | From Aggregator | Real-time (1 Hz) |
| `/mission/lockdown_command` | From mission control | When target locked |
| `/mission/kamikaze_command` | From mission control | When kamikaze mode activated |

#### What It Does

1. **Logs In** (at startup)
   - Sends username/password to Flask server
   - Receives authentication token

2. **Sends Telemetry** (0.6s interval)
   - Takes data from `/competition/telemetry_data`
   - Sends to server endpoint: `POST /api/telemetri_gonder`
   - Receives other UAVs' locations

3. **Fetches Data** (5s interval)
   - QR coordinates from `/api/qr_koordinati`
   - HSS zones from `/api/hss_koordinatlari`
   - Server time from `/api/sunucusaati`

4. **Handles Mission Commands**
   - Receives lockdown commands → Sends to `/api/kilitlenme_bilgisi`
   - Receives kamikaze commands → Sends to `/api/kamikaze_bilgisi`

#### What It Publishes (Output Topics)

Received data from server - published to these topics:

| Topic | Contains | From Server |
|-------|----------|-------------|
| `/competition/server_time` | Server time | `/api/sunucusaati` |
| `/competition/other_uavs` | Other UAVs location | Telemetry response |
| `/competition/qr_location` | QR code locations | `/api/qr_koordinati` |
| `/competition/hss_zones` | Safe zones | `/api/hss_koordinatlari` |
| `/competition/lockdown_info` | Lockdown status | Local feedback |
| `/competition/kamikaze_info` | Kamikaze status | Local feedback |

---

## 📊 Data Flow

### Normal Operation Flow

```
Every 1 second (1 Hz):
   PX4 Sensors
        ↓
   Aggregator (processes)
        ↓
   /competition/telemetry_data (published)
        ↓
   Bridge (picks up data)
        ↓
   Every 0.6s sends to Flask server
        ↓
   Server responds with other UAVs' data
        ↓
   /competition/other_uavs (published)

Every 5 seconds in parallel:
   Bridge fetches QR coordinates
        ↓
   /competition/qr_location (published)
   
   Bridge fetches HSS zones
        ↓
   /competition/hss_zones (published)
   
   Bridge fetches server time
        ↓
   /competition/server_time (published)
```

### When Mission Command Arrives

```
External Node sends:
   /mission/lockdown_command (with target data)
        ↓
   Bridge receives
        ↓
   Bridge processes JSON
        ↓
   Sends POST to /api/kilitlenme_bilgisi
        ↓
   Server processes
        ↓
   Bridge publishes to /competition/lockdown_info
```

---

## 📡 Topics Reference

### Complete Topic List

#### Published BY Aggregator
- **`/competition/telemetry_data`** - Raw telemetry (1 Hz) - All drone sensor data merged

#### Published BY Bridge
- **`/competition/server_time`** - Server's current time (5 Hz) - Updated from server every 5s
- **`/competition/other_uavs`** - Other UAVs positions - Updated from server every 0.6s
- **`/competition/qr_location`** - QR code coordinates (0.2 Hz) - Updated from server every 5s  
- **`/competition/hss_zones`** - No-fly zones (0.2 Hz) - Updated from server every 5s
- **`/competition/lockdown_info`** - Lockdown command echoes - When lockdown occurs
- **`/competition/kamikaze_info`** - Kamikaze command echoes - When kamikaze occurs

#### Subscribed BY Bridge
- **`/mission/lockdown_command`** - Commands from mission planner - JSON format
- **`/mission/kamikaze_command`** - Commands from mission planner - JSON format

---

## 💡 Examples

### Example 1: Monitor All Data

```bash
# Terminal 1: Start aggregator
ros2 run server_node telemetry_aggregator

# Terminal 2: Start bridge  
ros2 run server_node bridge

# Terminal 3: Watch telemetry data
ros2 topic echo /competition/telemetry_data --full-length

# Terminal 4: Watch server responses
ros2 topic echo /competition/other_uavs
```

### Example 2: Send a Lockdown Command

```bash
# Send a lockdown command (target locked at coordinates 100,200)
ros2 topic pub /mission/lockdown_command std_msgs/String \
  "{data: '{\"end_time\": {\"saat\": 12, \"dakika\": 10, \"saniye\": 15, \"milisaniye\": 500}, \"is_autonomous\": true, \"center_x\": 100, \"center_y\": 200, \"width\": 50, \"height\": 50}'}"

# Watch it get sent to server
ros2 topic echo /competition/lockdown_info
```

### Example 3: Send a Kamikaze Command

```bash
# Send a kamikaze mission
ros2 topic pub /mission/kamikaze_command std_msgs/String \
  "{data: '{\"start_time\": {\"saat\": 12, \"dakika\": 10, \"saniye\": 0, \"milisaniye\": 0}, \"end_time\": {\"saat\": 12, \"dakika\": 10, \"saniye\": 25, \"milisaniye\": 250}, \"qr_text\": \"TARGET_QR_123\"}'}"

# Watch it get sent to server
ros2 topic echo /competition/kamikaze_info
```

### Example 4: Debugging - See All Messages on All Topics

```bash
# Terminal 1
ros2 run server_node telemetry_aggregator

# Terminal 2
ros2 run server_node bridge

# Terminal 3: Monitor everything
for topic in $(ros2 topic list | grep competition); do
  echo "Monitoring: $topic"
  ros2 topic echo "$topic" --full-length 2>/dev/null &
done
wait
```

---

## 🔧 Telemetry Data Explained

Each piece of data in the telemetry packet:

```json
{
  "takim_numarasi": 1,              # Team ID (always 1)
  "iha_enlem": 41.508775,           # Latitude in degrees (-90 to +90)
  "iha_boylam": 36.118335,          # Longitude in degrees (-180 to +180)
  "iha_irtifa": 38.5,               # Altitude above sea level (meters)
  "iha_dikilme": 7.2,               # Roll (degrees, -90 to +90)
  "iha_yonelme": 210.5,             # Yaw/heading (degrees, 0 to 360, where 0=North)
  "iha_yatis": -30.1,               # Pitch (degrees, -90 to +90)
  "iha_hiz": 28.4,                  # Speed (meters/second)
  "iha_batarya": 85.5,              # Battery percentage (0 to 100)
  "iha_otonom": 1,                  # Autonomous mode? (0=manual, 1=autonomous)
  "iha_kilitlenme": 0,              # Target lock? (0=no, 1=yes)
  "hedef_merkez_X": 0,              # Target center X (pixels)
  "hedef_merkez_Y": 0,              # Target center Y (pixels)
  "hedef_genislik": 0,              # Target width (pixels)
  "hedef_yukseklik": 0,             # Target height (pixels)
  "gps_saati": {                    # GPS time of day
    "saat": 12,                     # Hour (0-23)
    "dakika": 10,                   # Minute (0-59)
    "saniye": 5,                    # Second (0-59)
    "milisaniye": 200               # Millisecond (0-999)
  }
}
```

---

## 🐛 Troubleshooting

### Node won't start

**Error**: `ModuleNotFoundError: No module named 'rclpy'`
```bash
# Solution: Install ROS2
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

**Error**: `Cannot find package 'px4_msgs'`
```bash
# Solution: Clone and build px4_msgs
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ..
colcon build --packages-select px4_msgs
source install/setup.bash
```

### No data appearing on topics

```bash
# Check if nodes are running
ros2 node list
# Should show: /telemetry_aggregator, /competition_bridge

# Check topics
ros2 topic list
# Should show all /competition/* topics

# Debug aggregator node
ros2 run server_node telemetry_aggregator --log-level DEBUG
```

### Server connection fails

```bash
# Check if server is running
curl http://127.0.0.1:5000/api/giris

# Check logs
ros2 run server_node bridge --log-level DEBUG
# Look for connection errors
```

### Telemetry data looks wrong

- **GPS shows 0,0**: Check PX4 SITL is publishing valid GPS
- **Battery shows 0**: Battery status may not be published
- **Orientation wrong**: May be quaternion conversion issue

```bash
# Debug aggregator data
ros2 topic echo /competition/telemetry_data --full-length
```

---

## 📁 File Structure

```
server_node/
├── server_node/
│   ├── __init__.py                      # Python package marker
│   ├── telemetry_aggregator_node.py     # Aggregator node (281 lines)
│   └── bridge.py                        # Bridge node (307 lines)
├── package.xml                          # ROS2 package manifest
├── setup.py                             # Python setup configuration
├── setup.cfg                            # Setup options
├── README.md                            # This file
└── resource/
    └── server_node                      # Package resource marker
```

---

## 📦 Dependencies

| Package | Purpose |
|---------|---------|
| `rclpy` | ROS2 Python client library |
| `std_msgs` | Standard ROS2 message types (String) |
| `px4_msgs` | PX4 drone message definitions |
| `requests` | HTTP library for Flask communication |
| `json` | JSON serialization (built-in Python) |
| `math` | Math functions (built-in Python) |

---

## 🔐 Configuration

Edit these in `server_node/bridge.py`:

```python
self.server_url = "http://127.0.0.1:5000"  # Flask server address
self.username = "zarqa"                     # Login username
self.password = "1234"                      # Login password
self.request_timeout = 5.0                  # Network timeout (seconds)
```

---

## 📖 Additional Resources

### For ROS2 Beginners
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Concepts](https://docs.ros.org/en/humble/Concepts/Basic.html)
- [Understanding Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

### For PX4 Developers
- [PX4 Topics](https://docs.px4.io/main/en/msg_docs/)
- [PX4 Messages](https://github.com/PX4/px4_msgs)
- [ROS2 with PX4](https://docs.px4.io/main/en/middleware/uxrce_dds.html)

### Quaternions Explained
- [Quaternion Tutorial](https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/)

---

## 📝 License

Apache License 2.0 - See LICENSE file for details

---

## ❓ Questions?

For issues with:
- **Architecture/Design**: Check the [Architecture](#architecture) section
- **ROS concepts**: See [What is ROS?](#what-is-ros)
- **Data formats**: See [Telemetry Data Explained](#telemetry-data-explained)
- **Running the system**: See [Running the System](#running-the-system)