# Competition Bridge Node

ROS2 (Humble) node that bridges communication between a ROS2 system and a Flask-based competition simulation server.

## Installation

### Prerequisites
- ROS2 Humble installed
- Python 3.10+
- requests library

### Setup

1. **Place the package** in your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   # server_node package should be here
   ```

2. **Install dependencies:**
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   pip install requests
   ```

3. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select server_node
   source install/setup.bash
   ```

## Configuration

Edit the following in [server_node/bridge.py](server_node/bridge.py):

- `server_url`: Flask server address (default: `http://127.0.0.1:5000`)
- `username`: Login username (default: `zarqa`)
- `password`: Login password (default: `1234`)
- `telemetry_data`: Static telemetry values including position, altitude, battery, etc.

## Running the Node

### In a single terminal:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run server_node bridge
```

### In multiple terminals (recommended):

**Terminal 1: Start ROS2 Domain:**
```bash
source ~/ros2_ws/install/setup.bash
```

**Terminal 2: Run the bridge node:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run server_node bridge
```

**Terminal 3: Monitor published topics:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /competition/server_time
ros2 topic echo /competition/other_uavs
ros2 topic echo /competition/qr_location
ros2 topic echo /competition/hss_zones
ros2 topic echo /competition/lockdown_info
ros2 topic echo /competition/kamikaze_info
```

## Features

### Authentication
- Logs in once at startup using `/api/giris` endpoint
- Credentials: `{"kadi": "zarqa", "sifre": "1234"}`
- Maintains session token for authenticated requests

### Telemetry Publishing (0.6s interval)
- **Sends** UAV telemetry to `/api/telemetri_gonder`
- **Receives** other UAVs location and publishes to `/competition/other_uavs`
- Includes required headers: `Content-Type: application/json`, `X-Kadi: zarqa`

### Data Fetching (5s interval)
- Fetches QR coordinates from `/api/qr_koordinati` → publishes to `/competition/qr_location`
- Fetches HSS zones from `/api/hss_koordinatlari` → publishes to `/competition/hss_zones`

### Lockdown Info Posting
- **Subscribes** to `/mission/lockdown_command` for real mission data
- **Posts** lockdown information to `/api/kilitlenme_bilgisi`
- **Publishes** lockdown events to `/competition/lockdown_info`
- Sends end time and autonomy flag to server
- Includes dummy data fallback for testing (disabled when real data is received)

### Kamikaze Info Posting
- **Subscribes** to `/mission/kamikaze_command` for real mission data
- **Posts** kamikaze information to `/api/kamikaze_bilgisi`
- **Publishes** kamikaze events to `/competition/kamikaze_info`
- Sends start time, end time, and QR text to server
- Includes dummy data fallback for testing (disabled when real data is received)

### Reliability
- 5-second request timeout on all HTTP calls
- Exception handling for network errors (connection, timeout, JSON parsing)
- Graceful degradation if server is unavailable
- Comprehensive logging with ROS2 logger
- Automatic mode switching between dummy and real data

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/competition/server_time` | std_msgs/String | Server time as JSON |
| `/competition/other_uavs` | std_msgs/String | Other UAVs location info as JSON |
| `/competition/qr_location` | std_msgs/String | QR coordinates as JSON |
| `/competition/hss_zones` | std_msgs/String | HSS zones data as JSON |
| `/competition/lockdown_info` | std_msgs/String | Lockdown event data as JSON |
| `/competition/kamikaze_info` | std_msgs/String | Kamikaze event data as JSON |

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mission/lockdown_command` | std_msgs/String | Lockdown mission commands (end time, autonomy) |
| `/mission/kamikaze_command` | std_msgs/String | Kamikaze mission commands (start/end time, QR text) |

## Telemetry Format

```json
{
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

## Lockdown Command Format

Publish to `/mission/lockdown_command` with this JSON format:

```json
{
  "end_time": {
    "saat": 12,
    "dakika": 10,
    "saniye": 15,
    "milisaniye": 500
  },
  "is_autonomous": true,
  "center_x": 100,
  "center_y": 200,
  "width": 50,
  "height": 50
}
```

## Kamikaze Command Format

Publish to `/mission/kamikaze_command` with this JSON format:

```json
{
  "start_time": {
    "saat": 12,
    "dakika": 10,
    "saniye": 0,
    "milisaniye": 0
  },
  "end_time": {
    "saat": 12,
    "dakika": 10,
    "saniye": 25,
    "milisaniye": 250
  },
  "qr_text": "TARGET_QR_CODE"
}
```

## Testing Mission Commands

### Publish Lockdown Command
```bash
ros2 topic pub /mission/lockdown_command std_msgs/String "{data: '{\"end_time\": {\"saat\": 12, \"dakika\": 10, \"saniye\": 15, \"milisaniye\": 500}, \"is_autonomous\": true}'}"
```

### Publish Kamikaze Command
```bash
ros2 topic pub /mission/kamikaze_command std_msgs/String "{data: '{\"start_time\": {\"saat\": 12, \"dakika\": 10, \"saniye\": 0, \"milisaniye\": 0}, \"end_time\": {\"saat\": 12, \"dakika\": 10, \"saniye\": 25, \"milisaniye\": 250}, \"qr_text\": \"TEST_QR\"}'}"
```

## Troubleshooting

### Node fails to start
```bash
# Check ROS2 installation
ros2 doctor
```

### Cannot connect to server
- Verify Flask server is running at `http://127.0.0.1:5000`
- Check firewall settings
- Verify network connectivity

### Dummy data still sending
- Ensure you're publishing to `/mission/lockdown_command` and `/mission/kamikaze_command` topics
- The node automatically detects real data and disables dummy data

### Check logs
```bash
ros2 run server_node bridge --log-level DEBUG
```

## File Structure

```
server_node/
├── server_node/
│   ├── __init__.py
│   └── bridge.py          # Main ROS2 node
├── package.xml            # ROS2 package metadata
├── setup.py              # Python setup configuration
├── setup.cfg             # Setup configuration
└── README.md             # This file
```

## Dependencies

- `rclpy`: ROS2 Python client library
- `std_msgs`: Standard ROS2 message types
- `requests`: HTTP library for Flask communication

## License

Apache License 2.0