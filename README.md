# Server Node - ROS2 Bridge for Competition Simulation

High-performance ROS2 bridge connecting PX4 SITL drone simulator to Flask competition server via modular telemetry aggregation pipeline.

---

## 💾 Installation & Build

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip install requests

# Build with COLCON
colcon build --packages-select server_node --symlink-install

source install/setup.bash
```

### Requirements

- ROS2 Humble
- Python 3.10+
- px4_msgs (auto-installed from ROS index)
- requests library

---

## 🚀 System Architecture

### Component Layout

```
PX4 SITL (/fmu/out/*)
    ↓
┌──────────────────────────────────┐
│ TelemetryAggregatorNode          │
│ ────────────────────────────      │
│ • Quaternion → Euler conversion  │
│ • Velocity selection (v_ground/v_air)
│ • GPS format validation          │
│ • Time conversion (µs → dict)    │
│                                  │
│ QoS: BEST_EFFORT, depth=10       │
│ Publish freq: 1 Hz               │
└──────────────────────────────────┘
    ↓ (/competition/telemetry_data)
┌──────────────────────────────────┐
│ CompetitionBridgeNode            │
│ ────────────────────────────      │
│ • HTTP/Auth to Flask             │
│ • Telemetry send (0.6 Hz)        │
│ • Data fetch (0.2 Hz: QR, HSS)   │
│ • Mission commands (async)       │
│                                  │
│ Features:                         │
│ - 5s request timeout             │
│ - Automatic reconnection         │
│ - JSON frame encoding            │
└──────────────────────────────────┘
    ↓ (HTTP + ROS topics)
Flask Server (127.0.0.1:5000)
```

---

## 📋 Technical Specifications

### Telemetry Aggregator Node

**File**: `server_node/telemetry_aggregator_node.py`  
**Process**: PX4 sensor fusion & computation  
**Launch**: `ros2 run server_node telemetry_aggregator`

#### Input Topics (PX4 SITL)

| Topic | Message Type | Rate | Fields Used |
|-------|--------------|------|------------|
| `/fmu/out/vehicle_attitude` | `VehicleAttitude` | ~50 Hz | `q[4]` (quaternion) |
| `/fmu/out/vehicle_local_position_v1` | `VehicleLocalPosition` | ~50 Hz | `vx, vy, vz` (m/s) |
| `/fmu/out/vehicle_global_position` | `VehicleGlobalPosition` | ~25 Hz | `lat, lon, alt, timestamp` |
| `/fmu/out/battery_status_v1` | `BatteryStatus` | ~1 Hz | `remaining` |
| `/fmu/out/vehicle_status` | `VehicleStatus` | ~5 Hz | `nav_state` |
| `/fmu/out/airspeed_validated_v1` | `AirspeedValidated` | ~10 Hz | `true_airspeed_m_s` |

#### Processing Pipeline

```python
# Quaternion conversion (Hamilton convention)
q = [x, y, z, w]
roll = atan2(2(wx + yz), 1 - 2(x² + y²))
pitch = asin(2(wy - zx))
yaw = atan2(2(wz + xy), 1 - 2(y² + z²))

# Velocity selection logic
if ground_velocity > 0.5 m/s:
    use ground_speed
elif airspeed > 0:
    use airspeed  # SITL fallback
else:
    use ground_speed

# GPS coordinate detection (auto-scaling)
if |lat| > 1000 OR |lon| > 1000:
    # Real hardware format (degrees × 1e7)
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
else:
    # SITL format (degrees)
    lat = msg.lat
    lon = msg.lon

# Time conversion (microseconds → struct)
t_days = floor(µs / (24*3600*1e6))
t_mod = µs % (24*3600*1e6)
hours = floor(t_mod / 3600e6) % 24
```

#### Output Topic

**Topic**: `/competition/telemetry_data`  
**Type**: `std_msgs/String` (JSON serialized)  
**Frequency**: 1 Hz  
**QoS**: BEST_EFFORT, depth=10

```json
{
  "takim_numarasi": 1,
  "iha_enlem": <float -90..90>,
  "iha_boylam": <float -180..180>,
  "iha_irtifa": <float ≥0>,
  "iha_dikilme": <float -90..90>,
  "iha_yonelme": <float 0..360>,
  "iha_yatis": <float -90..90>,
  "iha_hiz": <float ≥0>,
  "iha_batarya": <float 0..100>,
  "iha_otonom": <int 0|1>,
  "iha_kilitlenme": <int 0|1>,
  "hedef_merkez_X": <int>,
  "hedef_merkez_Y": <int>,
  "hedef_genislik": <int>,
  "hedef_yukseklik": <int>,
  "gps_saati": {
    "saat": <int 0..23>,
    "dakika": <int 0..59>,
    "saniye": <int 0..59>,
    "milisaniye": <int 0..999>
  }
}
```

---

## 🔍 Telemetry Data Extraction Map

This section documents exactly how each telemetry field is extracted from PX4 topics, enabling reproduction on real hardware.

### Topic-to-Telemetry Mapping Reference

| Telemetry Field | Source Topic | Message Type | Field Path | Callback Function | Transformation |
|-----------------|--------------|--------------|------------|-------------------|-----------------|
| `iha_enlem` (latitude) | `/fmu/out/vehicle_global_position` | `VehicleGlobalPosition` | `msg.lat` | `global_position_callback()` | Auto-scaled: if > 1000 → divide by 1e7, else use raw |
| `iha_boylam` (longitude) | `/fmu/out/vehicle_global_position` | `VehicleGlobalPosition` | `msg.lon` | `global_position_callback()` | Auto-scaled: if > 1000 → divide by 1e7, else use raw |
| `iha_irtifa` (altitude) | `/fmu/out/vehicle_global_position` | `VehicleGlobalPosition` | `msg.alt` | `global_position_callback()` | Direct pass, bounds: [0, ∞) |
| `gps_saati` (time struct) | `/fmu/out/vehicle_global_position` | `VehicleGlobalPosition` | `msg.timestamp` | `global_position_callback()` | Microseconds → {saat, dakika, saniye, milisaniye} |
| `iha_dikilme` (roll) | `/fmu/out/vehicle_attitude` | `VehicleAttitude` | `msg.q[4]` | `attitude_callback()` | Quaternion→Euler (Hamilton), degrees, bounds: [-90, 90] |
| `iha_yonelme` (yaw/heading) | `/fmu/out/vehicle_attitude` | `VehicleAttitude` | `msg.q[4]` | `attitude_callback()` | Quaternion→Euler (Hamilton), degrees, bounds: [0, 360] |
| `iha_yatis` (pitch) | `/fmu/out/vehicle_attitude` | `VehicleAttitude` | `msg.q[4]` | `attitude_callback()` | Quaternion→Euler (Hamilton), degrees, bounds: [-90, 90] |
| `iha_hiz` (velocity) | `/fmu/out/vehicle_local_position_v1` `/fmu/out/airspeed_validated_v1` | `VehicleLocalPosition` `AirspeedValidated` | `msg.vx, msg.vy` `msg.true_airspeed_m_s` | `local_position_callback()` | Ground speed from sqrt(vx²+vy²); fallback to airspeed if < 0.5 m/s |
| `iha_batarya` (battery) | `/fmu/out/battery_status_v1` | `BatteryStatus` | `msg.remaining` | `battery_callback()` | Clamp [0,1] → multiply by 100 → range [0, 100+] |
| `iha_otonom` (autonomous) | `/fmu/out/vehicle_status` | `VehicleStatus` | `msg.nav_state` | `vehicle_status_callback()` | 1 if nav_state ∈ {3, 14}, else 0 |
| `iha_kilitlenme` (locked) | (Internal state) | — | — | `update_lockdown_target()` | Set 1 on lockdown, 0 on release |
| `hedef_merkez_*` (target) | (Internal state) | — | — | `update_lockdown_target()` | Set by mission target or vision system |

---

### Detailed Extraction Procedures

#### 1. Global Position & GPS Time (`/fmu/out/vehicle_global_position`)

**Message Type**: `px4_msgs/msg/VehicleGlobalPosition`

**Subscription QoS**: BEST_EFFORT, depth=10  
**Expected Rate**: 25-30 Hz (decimated to 1 Hz in aggregator)

**Extraction Logic** (`global_position_callback`):
```python
# Raw message fields from PX4
lat = msg.lat          # GPS latitude (int32)
lon = msg.lon          # GPS longitude (int32)
alt = msg.alt          # Altitude MSL in meters (float)
timestamp = msg.timestamp  # Time in microseconds (uint64)

# Handle SITL vs Real Hardware coordinate scaling
if abs(lat) > 1000 or abs(lon) > 1000:
    # Real hardware: coordinates stored as degrees × 1e7
    latitude = lat / 1e7
    longitude = lon / 1e7
else:
    # SITL simulation: coordinates already in degrees
    latitude = float(lat)
    longitude = float(lon)

# Bounds enforcement
latitude = max(-90, min(90, latitude))
longitude = max(-180, min(180, longitude))
altitude = max(0, altitude)

# Output assignments
telemetry["iha_enlem"] = round(latitude, 6)      # 6 decimals ≈ 0.1m resolution
telemetry["iha_boylam"] = round(longitude, 6)
telemetry["iha_irtifa"] = round(altitude, 2)
telemetry["gps_saati"] = gps_time_to_dict(timestamp)
```

**GPS Time Conversion**:
```python
# Convert microsecond timestamp to human-readable time
total_seconds = gps_time_us / 1_000_000
hours = int((total_seconds / 3600) % 24)      # 0-23
minutes = int((total_seconds / 60) % 60)      # 0-59
seconds = int(total_seconds % 60)             # 0-59
milliseconds = int((gps_time_us % 1_000_000) / 1000)  # 0-999

# Result: {"saat": h, "dakika": m, "saniye": s, "milisaniye": ms}
```

**Real Hardware Notes**:
- GPS coordinates may be scaled by 1e7 (WGS84 standard for int32 storage)
- Auto-detection in code checks if abs(coordinate) > 1000 to distinguish formats
- Timestamp is GPS time since epoch (Jan 6, 1980), not Unix time

---

#### 2. Vehicle Attitude & Orientation (`/fmu/out/vehicle_attitude`)

**Message Type**: `px4_msgs/msg/VehicleAttitude`

**Subscription QoS**: BEST_EFFORT, depth=10  
**Expected Rate**: 50-100 Hz (decimated to 1 Hz in aggregator)

**Extraction Logic** (`attitude_callback`):
```python
# Raw quaternion from PX4 (Hamilton convention: wxyz)
q = msg.q              # [q[0]=x, q[1]=y, q[2]=z, q[3]=w]
w, x, y, z = q[3], q[0], q[1], q[2]

# Quaternion → Euler conversion (Hamilton convention)
sinr_cosp = 2 * (w*x + y*z)
cosr_cosp = 1 - 2 * (x*x + y*y)
roll = atan2(sinr_cosp, cosr_cosp)

sinp = 2 * (w*y - z*x)
if abs(sinp) >= 1:
    pitch = copysign(π/2, sinp)  # Handle singularity
else:
    pitch = asin(sinp)

siny_cosp = 2 * (w*z + x*y)
cosy_cosp = 1 - 2 * (y*y + z*z)
yaw = atan2(siny_cosp, cosy_cosp)

# Convert radians to degrees and normalize
roll_deg = degrees(roll)           # Range: [-180, 180]
pitch_deg = degrees(pitch)         # Range: [-90, 90]
yaw_deg = degrees(yaw)             # Range: [-180, 180]

# Normalize yaw to [0, 360]
yaw_deg = yaw_deg % 360
if yaw_deg < 0:
    yaw_deg += 360

# Clamp all angles to expected ranges
roll_deg = max(-90, min(90, roll_deg))
pitch_deg = max(-90, min(90, pitch_deg))
yaw_deg = max(0, min(360, yaw_deg))

# Output assignments
telemetry["iha_dikilme"] = round(roll_deg, 2)      # Roll (negative = left wing down)
telemetry["iha_yonelme"] = round(yaw_deg, 2)       # Yaw (0=North, 90=East, 180=South, 270=West)
telemetry["iha_yatis"] = round(pitch_deg, 2)       # Pitch (negative = nose down)
```

**Gimbal Lock Handling**:
- When pitch ≈ ±90°, yaw becomes undefined (gimbal lock singularity)
- Code uses `copysign(π/2, sinp)` to preserve sign at boundary
- For high-precision attitude during lock: store raw quaternion separately if needed

**Real Hardware Calibration**:
- Requires IMU calibration before flight
- Quaternion output depends on sensor fusion algorithm (usually EKF2 or EKF3 in PX4)
- Yaw reference is magnetic north (requires compass calibration)

---

#### 3. Local Velocity (`/fmu/out/vehicle_local_position_v1`)

**Message Type**: `px4_msgs/msg/VehicleLocalPosition`

**Subscription QoS**: BEST_EFFORT, depth=10  
**Expected Rate**: 50-100 Hz (decimated to 1 Hz in aggregator)

**Extraction Logic** (`local_position_callback`):
```python
# Velocity components in NED (North-East-Down) frame
vx = msg.vx             # North velocity (m/s)
vy = msg.vy             # East velocity (m/s)
vz = msg.vz             # Down velocity (m/s) - positive = descending

# Calculate ground speed (horizontal velocity magnitude)
ground_speed = sqrt(vx^2 + vy^2)  # Ignores vertical component
ground_speed = max(0, ground_speed)

# Velocity selection logic with airspeed fallback
if ground_speed > 0.5:
    # Reliable ground speed data available
    selected_velocity = ground_speed
    source = "ground_speed"
elif last_airspeed > 0:
    # Hovering or low-speed: fallback to airspeed
    selected_velocity = last_airspeed
    source = "airspeed_fallback"
else:
    # No airspeed data: use ground speed anyway
    selected_velocity = ground_speed
    source = "ground_speed_only"

# Output assignment
telemetry["iha_hiz"] = round(selected_velocity, 2)
```

**Real Hardware Considerations**:
- Ground speed may become unreliable during GPS signal loss
- In SITL: airspeed > 0 is usually preferred over ground speed at low speeds
- In real flight: ground speed is typically more reliable once airspeed > 5 m/s

---

#### 4. Battery Status (`/fmu/out/battery_status_v1`)

**Message Type**: `px4_msgs/msg/BatteryStatus`

**Subscription QoS**: BEST_EFFORT, depth=10  
**Expected Rate**: 1-5 Hz

**Extraction Logic** (`battery_callback`):
```python
# Raw battery remaining percentage (0.0 to 1.0)
battery_remaining = msg.remaining

# Clamp to valid range and convert percentage
battery_clamped = max(0.0, min(1.0, battery_remaining))
battery_percent = battery_clamped * 100

# Output assignment (Note: May exceed 100 if raw data unreliable)
telemetry["iha_batarya"] = round(battery_percent, 2)
```

**Calibration Notes**:
- Requires battery capacity calibration in PX4
- May read 100-110% if voltage is higher than expected
- May not track discharge accurately until first ~10% used
- Restart often resets to 100% regardless of actual charge

**Real Hardware Cautions**:
- Battery estimation depends on accurate capacity configuration
- Voltage offset can cause 20-30% reading errors if miscalibrated
- Multiple battery packs may have different calibration values

---

#### 5. Autonomous Mode Status (`/fmu/out/vehicle_status`)

**Message Type**: `px4_msgs/msg/VehicleStatus`

**Subscription QoS**: BEST_EFFORT, depth=10  
**Expected Rate**: 5-10 Hz

**Extraction Logic** (`vehicle_status_callback`):
```python
# Navigation state determines if autonomous
nav_state = msg.nav_state  # Integer enum value

# Autonomous modes in PX4
AUTONOMOUS_NAV_STATES = {
    3: "Auto.Mission",         # Executing mission
    14: "Auto.Loiter"          # Loitering/holding position
}

# Determine autonomous flag
is_autonomous = nav_state in AUTONOMOUS_NAV_STATES

# Output assignment
telemetry["iha_otonom"] = 1 if is_autonomous else 0
```

**PX4 Navigation States**:
```
0: Manual              - Direct RC control
1: Altitude Control   - Altitude held via RC
2: Position Control   - Position held via RC
3: Mission            - Autonomous mission
4: Velocity Control   - Velocity commanded
5: Acro               - Aerobatic (manual rates)
6: Offboard           - External command via MAVLink
14: Loiter            - Autonomous loitering
(others: stabilized modes, RTH, takeoff, land, etc.)
```

**Real Hardware Mapping**:
- Mission flights: nav_state = 3
- Hovering at waypoint: nav_state = 14
- Auto-return-to-home: nav_state ∉ {3, 14} → not autonomous
- Manual RC control: nav_state = 0 → not autonomous

---

#### 6. Airspeed Backup (`/fmu/out/airspeed_validated_v1`)

**Message Type**: `px4_msgs/msg/AirspeedValidated`

**Subscription QoS**: BEST_EFFORT, depth=10  
**Expected Rate**: 10-20 Hz (used only as fallback)

**Extraction Logic** (`airspeed_callback`):
```python
# True airspeed reading in m/s
true_airspeed = msg.true_airspeed_m_s

# Cache for fallback use in velocity selection
last_airspeed = max(0.0, true_airspeed)

# Note: This airspeed is NOT stored directly in telemetry
# Instead, it's used as fallback when ground_speed < 0.5 m/s
```

**When Airspeed Is Used**:
- SITL with low ground speeds: frequently used (motor noise appears as altitude/velocity)
- Real flight below 5 m/s: airspeed often more stable
- High-altitude flight: airspeed less affected by wind filter delays

**Limitations**:
- Requires pitot tube calibration
- Susceptible to ice/rain (real aircraft)
- May lag during rapid speed changes
- Zero reading possible near hover in SITL

---

### Sequence Diagram: Data Flow

```
PX4 SITL Sensors (10-100 Hz)
    ↓
    ├─ IMU (200 Hz) → EKF Estimate
    │   ↓
    │   └─ /fmu/out/vehicle_attitude (50 Hz)
    │       VehicleAttitude.q[4] (quaternion)
    │       └─ attitude_callback()
    │           ├─ quaternion_to_euler()
    │           ├─ Normalize angles: roll[-90,90], pitch[-90,90], yaw[0,360]
    │           └─ Update: iha_dikilme, iha_yonelme, iha_yatis
    │
    ├─ GPS/Vision (25 Hz)
    │   ↓
    │   └─ /fmu/out/vehicle_global_position (25 Hz)
    │       VehicleGlobalPosition (lat, lon, alt, timestamp)
    │       └─ global_position_callback()
    │           ├─ Auto-detect scaling (SITL vs Hardware)
    │           ├─ gps_time_to_dict(microseconds)
    │           └─ Update: iha_enlem, iha_boylam, iha_irtifa, gps_saati
    │
    ├─ State Estimator (50 Hz)
    │   ↓
    │   └─ /fmu/out/vehicle_local_position_v1 (50 Hz)
    │       VehicleLocalPosition (vx, vy, vz)
    │       └─ local_position_callback()
    │           ├─ Calculate: ground_speed = sqrt(vx² + vy²)
    │           ├─ Check: ground_speed > 0.5 m/s ?
    │           ├─ If yes: use ground_speed
    │           ├─ If no: use last_airspeed (fallback)
    │           └─ Update: iha_hiz
    │
    ├─ Battery Monitor (2 Hz)
    │   ↓
    │   └─ /fmu/out/battery_status_v1 (2 Hz)
    │       BatteryStatus (remaining: 0.0-1.0)
    │       └─ battery_callback()
    │           ├─ Clamp: [0, 1]
    │           ├─ Scale: × 100
    │           └─ Update: iha_batarya
    │
    ├─ Navigation Stack (5 Hz)
    │   ↓
    │   └─ /fmu/out/vehicle_status (5 Hz)
    │       VehicleStatus (nav_state: enum)
    │       └─ vehicle_status_callback()
    │           ├─ Check: nav_state ∈ {3, 14} (mission/loiter)
    │           └─ Update: iha_otonom (0 or 1)
    │
    └─ Airspeed Sensor (10 Hz)
        ↓
        └─ /fmu/out/airspeed_validated_v1 (10 Hz)
            AirspeedValidated (true_airspeed_m_s)
            └─ airspeed_callback()
                ├─ Store: last_airspeed (used by local_position_callback)
                └─ (not written directly to telemetry)

                All callbacks update: self.telemetry_data{}
                    ↓
                Timer (1 Hz)
                    ↓
                publish_telemetry_callback()
                    ├─ json.dumps(self.telemetry_data)
                    └─ Publish to /competition/telemetry_data
                        ↓
                        Bridge Node subscribes
                        ↓
                        Sends to Flask server @ 0.6 Hz
```

---

### Implementation Checklist for Real Hardware

When adapting this code to real aircraft hardware:

**Coordinate Systems**:
- [ ] Verify latitude/longitude scaling (1e7 for real hardware vs raw for SITL)
- [ ] Confirm altitude reference (MSL vs AGL)
- [ ] Check velocity frame (NED is standard for VehicleLocalPosition)

**Sensor Calibration**:
- [ ] Compass calibration (affects yaw/heading)
- [ ] IMU accelerometer & gyro calibration (affects roll/pitch/yaw)
- [ ] Battery capacity configuration (affects battery percentage reading)
- [ ] Airspeed sensor calibration if using independent pitot tube

**Topic Rates & Timing**:
- [ ] Verify actual publish rates match expectations (25-30 Hz for GPS)
- [ ] Check for message drops in logs (record with rosbag)
- [ ] Monitor aggregator CPU usage at 1 Hz output

**Edge Cases**:
- [ ] GPS loss (will repeat last-known position)
- [ ] Battery disconnect mid-flight (may drop to ~0% suddenly)
- [ ] Magnetic interference (yaw drift without compass calibration)
- [ ] Gimbal lock near 90° pitch (yaw undefined in Euler representation)

---

**File**: `server_node/bridge.py`  
**Process**: Server communication & command routing  
**Launch**: `ros2 run server_node bridge`

#### Input Topics

| Topic | Type | Format | Handling |
|-------|------|--------|----------|
| `/competition/telemetry_data` | String | JSON | Cached, sent at 0.6 Hz |
| `/mission/lockdown_command` | String | JSON | Direct call to `/api/kilitlenme_bilgisi` |
| `/mission/kamikaze_command` | String | JSON | Direct call to `/api/kamikaze_bilgisi` |

#### HTTP API Integration

```
POST /api/giris (startup)
├─ payload: {"kadi": "zarqa", "sifre": "1234"}
└─ response: token info (unused in current impl)

POST /api/telemetri_gonder (0.6 Hz)
├─ payload: <telemetry_data JSON>
├─ timeout: 5s
├─ headers: {"X-Kadi": "zarqa", "Content-Type": "application/json"}
└─ response: {"sunucusaati": {...}, "konumBilgileri": [...]}

GET /api/qr_koordinati (5s interval)
├─ timeout: 5s
├─ headers: {"X-Kadi": "zarqa"}
└─ response: QR coordinates JSON

GET /api/hss_koordinatlari (5s interval)
├─ timeout: 5s
├─ headers: {"X-Kadi": "zarqa"}
└─ response: HSS zones JSON

GET /api/sunucusaati (5s interval)
├─ timeout: 5s
├─ headers: {"X-Kadi": "zarqa"}
└─ response: Server time JSON

POST /api/kilitlenme_bilgisi (async)
├─ payload: {"kilitlenmeBitisZamani": {...}, "otonom_kilitlenme": <0|1>}
└─ timeout: 5s

POST /api/kamikaze_bilgisi (async)
├─ payload: {"kamikazeBaslangicZamani": {...}, "kamikazeBitisZamani": {...}, "qrMetni": string}
└─ timeout: 5s
```

#### Output Topics

| Topic | Frequency | Source |
|-------|-----------|--------|
| `/competition/server_time` | 5 Hz | GET `/api/sunucusaati` response |
| `/competition/other_uavs` | 0.6 Hz | POST `/api/telemetri_gonder` response |
| `/competition/qr_location` | 0.2 Hz | GET `/api/qr_koordinati` response |
| `/competition/hss_zones` | 0.2 Hz | GET `/api/hss_koordinatlari` response |
| `/competition/lockdown_info` | event | POST `/api/kilitlenme_bilgisi` |
| `/competition/kamikaze_info` | event | POST `/api/kamikaze_bilgisi` |

---

## � Full Telemetry Schema

**Message Type**: `std_msgs/String` (JSON)  
**Topic**: `/competition/telemetry_data`  
**Publish Rate**: 1 Hz  
**QoS Settings**: 
- Reliability: BEST_EFFORT
- Durability: VOLATILE  
- History depth: 10 messages

### Field Specifications

| Field | Type | Range | Source | Notes |
|-------|------|-------|--------|-------|
| `takim_numarsi` | int | 0-255 | config | Team ID for server identification |
| `iha_enlem` | float | -90.0 to 90.0 | `/fmu/out/vehicle_global_position` lat | Latitude in decimal degrees; auto-scaled from hardware/SITL |
| `iha_boylam` | float | -180.0 to 180.0 | `/fmu/out/vehicle_global_position` lon | Longitude in decimal degrees |
| `iha_irtifa` | float | ≥ 0 | `/fmu/out/vehicle_global_position` alt | Altitude in meters MSL |
| `iha_dikilme` | float | -90.0 to 90.0 | quaternion_to_euler(pitch) | Pitch (nose up/down) in degrees |
| `iha_yonelme` | float | 0.0 to 360.0 | quaternion_to_euler(yaw) | Yaw (heading) in degrees [0=North, 90=East, 180=South, 270=West] |
| `iha_yatis` | float | -90.0 to 90.0 | quaternion_to_euler(roll) | Roll (wing bank) in degrees; negative=left |
| `iha_hiz` | float | ≥ 0 | velocity selection logic | Ground speed (m/s) preferred; airspeed fallback if ground speed < 0.5 m/s |
| `iha_batarya` | float | 0 to 100+ | `/fmu/out/battery_status_v1` remaining | Battery remaining % (NOTE: Multiplied by 100 after clamping [0,1] - may exceed 100) |
| `iha_otonom` | int | 0 \| 1 | `/fmu/out/vehicle_status` nav_state | 1 = autonomous mode, 0 = manual mode |
| `iha_kilitlenme` | int | 0 \| 1 | internal state | 1 = locked (stationary), 0 = flying |
| `hedef_merkez_X` | int | ≥ 0 | target tracking | X coordinate of target center (pixels) |
| `hedef_merkez_Y` | int | ≥ 0 | target tracking | Y coordinate of target center (pixels) |
| `hedef_genislik` | int | ≥ 0 | target tracking | Target bounding box width (pixels) |
| `hedef_yukseklik` | int | ≥ 0 | target tracking | Target bounding box height (pixels) |
| `gps_saati` | struct | — | `/fmu/out/vehicle_global_position` timestamp | Converted from microseconds to {saat, dakika, saniye, milisaniye} |

#### Quaternion-to-Euler Conversion

Implementation uses Hamilton convention (wxyz):

```python
def quaternion_to_euler(q_x, q_y, q_z, q_w):
    # q = (x, y, z, w)
    roll = math.atan2(2*(q_w*q_x + q_y*q_z), 1 - 2*(q_x**2 + q_y**2))
    pitch = math.asin(2*(q_w*q_y - q_z*q_x))
    yaw = math.atan2(2*(q_w*q_z + q_x*q_y), 1 - 2*(q_y**2 + q_z**2))
    
    # Convert radians to degrees
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
```

**Singularity**: Pitch ≈ ±90° → gimbal lock. Yaw becomes undefined; use raw quaternion for this edge case.

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

## 🚀 Usage Examples

### Example 1: Testing Message Publication Rates

```bash
# Start both nodes in separate terminals
ros2 run server_node telemetry_aggregator
ros2 run server_node bridge

# Measure aggregator publishing frequency (should be ~1 Hz)
ros2 topic hz /competition/telemetry_data
# Output: average rate: 1.000
#         min: 1.002s max: 1.008s std dev: 0.001s

# Measure telemetry transmission rate to server (0.6 Hz approximately)
ros2 topic hz /competition/other_uavs
# Output: average rate: 0.600
```

### Example 2: Analyzing Telemetry JSON Structure

```bash
# Capture and parse telemetry (extract only orientation)
ros2 topic echo /competition/telemetry_data --full-length | \
  python3 -c "
import sys, json
for line in sys.stdin:
    if line.startswith('{'):
        msg = json.loads(line.replace(\"data: '\", '').rstrip(\"'\\n\"))
        print(f\"Roll: {msg['iha_yatis']}, Pitch: {msg['iha_dikilme']}, Yaw: {msg['iha_yonelme']}\")
"

# Example output:
# Roll: 5.2, Pitch: -2.1, Yaw: 127.3
# Roll: 5.3, Pitch: -2.0, Yaw: 128.1
```

### Example 3: Latency Monitoring

```bash
# Measure end-to-end latency: PX4 sensor → aggregator → bridge → server

# In aggregator_node.py, add timestamp at publish
# In bridge.py, check receive-to-send interval

ros2 topic echo /competition/telemetry_data --full-length 1 | \
  head -1 | python3 -c "
import sys, json, time
msg = json.loads(sys.stdin.read().replace(\"data: '\", '').rstrip(\"'\\n\"))
print(f'GPS time from msg: {msg[\"gps_saati\"]}')
print(f'Local time: {time.time()}')
print('Telemetry latency < 1 second expected')
"
```

### Example 4: Mission Command Integration Testing

```bash
# Terminal 1
ros2 run server_node telemetry_aggregator

# Terminal 2
ros2 run server_node bridge

# Terminal 3: Send lockdown command with proper JSON
ros2 topic pub /mission/lockdown_command std_msgs/String \
  '{data: "{\"kilitlenmeBitisZamani\": {\"saat\": 14, \"dakika\": 30, \"saniye\": 45, \"milisaniye\": 500}, \"otonom_kilitlenme\": 1, \"hedef_merkez_X\": 240, \"hedef_merkez_Y\": 160, \"hedef_genislik\": 80, \"hedef_yukseklik\": 60}"}'

# Terminal 4: Monitor command echo
ros2 topic echo /competition/lockdown_info
```

### Example 5: Direct HTTP Testing (Bypass ROS)

```bash
# Test server API directly
curl -X POST http://127.0.0.1:5000/api/giris \
  -H "Content-Type: application/json" \
  -d '{"kadi": "zarqa", "sifre": "1234"}'

# Manually send telemetry
curl -X POST http://127.0.0.1:5000/api/telemetri_gonder \
  -H "X-Kadi: zarqa" \
  -H "Content-Type: application/json" \
  -d '{
    "takim_numarasi": 1,
    "iha_enlem": 41.508775,
    "iha_boylam": 36.118335,
    "iha_irtifa": 50.0,
    "iha_hiz": 15.5
  }'
```

### Example 6: Recording and Playback

```bash
# Record 60 seconds of all telemetry
ros2 bag record /competition/telemetry_data /competition/other_uavs \
  --max-bag-size 100000000 -o flight_log

# Playback with rate scaling (2x speed)
ros2 bag play flight_log --rate 2.0

# Extract telemetry to CSV
ros2 bag info flight_log
sqlite3 flight_log/flight_log.db \
  "SELECT * FROM messages WHERE topic_id=1" > telemetry.csv
```

---

## 🔧 Advanced Configuration

### Node Timing Tuning

**Aggregator Publishing Rate**: Edit [telemetry_aggregator_node.py](server_node/telemetry_aggregator_node.py#L21)
```python
timer_period = 1.0  # Change from 1.0 Hz to desired rate
```

**Bridge Telemetry Send Rate**: Edit [bridge.py](server_node/bridge.py#L85)
```python
self.telemetry_timer = self.create_timer(1.666, self.send_telemetry_callback)  # 0.6 Hz → 1.666s
```

**Bridge Fetch Rate**: Edit [bridge.py](server_node/bridge.py#L95)
```python
self.fetch_timer = self.create_timer(5.0, self.fetch_qr_hss_callback)  # 0.2 Hz → 5.0s interval
```

### JSON Validation

Aggregator validates output JSON before publishing:
```python
# Bounds checking
iha_enlem = max(-90, min(90, lat))
iha_boylam = max(-180, min(180, lon))
iha_irtifa = max(0, altitude)
iha_hiz = max(0, velocity)
iha_batarya = max(0, min(100, percent)) * 100  # Clamped range [0-100]
```

Bridge validates server requests before sending:
```python
json_data = json.loads(msg.data)  # Raises JSONDecodeError if invalid
assert json_data['takim_numarasi'] > 0
```

### Error Handling

**Bridge HTTP Error Strategy**:
- `requests.Timeout` (5s) → Log & retry next cycle
- `requests.ConnectionError` → Log & retry next cycle  
- `json.JSONDecodeError` → Log warning, use cached data
- `KeyError` in telemetry → Default to 0, log error

---

## 🐛 Diagnostics

### Debugging Node Startup

**Import Errors**:
```bash
python3 -c "import rclpy; print('ROS2 OK')"
python3 -c "import px4_msgs; print('PX4_msgs OK')"
python3 -c "import requests; print('requests OK')"
```

**ROS Domain ID Mismatch**:
```bash
# Ensure both nodes use same domain (default 0)
export ROS_DOMAIN_ID=0
ros2 topic list  # Verify all topics visible

# Different domains won't communicate
export ROS_DOMAIN_ID=1
ros2 topic list  # Won't see any /competition/* topics
```

### Message Flow Validation

**Check aggregator is publishing**:
```bash
ros2 node list | grep telemetry
ros2 topic echo /competition/telemetry_data --full-length 1

# Verify JSON structure
ros2 topic bw /competition/telemetry_data
# Should show ~500 bytes/sec at 1 Hz
```

**Check bridge is receiving and forwarding**:
```bash
# Verify bridge is subscribed
ros2 node info /competition_bridge | grep Subscriptions

# Monitor server connection in logs
ros2 run server_node bridge --log-level DEBUG 2>&1 | \
  grep -E "Connected|timeout|error"
```

**Validate HTTP connectivity**:
```bash
# Test Flask server directly
nc -zv 127.0.0.1 5000
curl -v http://127.0.0.1:5000/api/giris

# If connection fails, check server is running
ps aux | grep flask
```

### Performance Analysis

**Latency Measurement**:
```bash
# Capture telemetry with timestamps
ros2 topic echo /competition/telemetry_data --csv > telemetry.csv
# Calculate µs → publish lag from GPS time

# Monitor single cycle timing
ros2 run server_node telemetry_aggregator --log-level INFO 2>&1 | \
  tail -20 | grep -E "time|published"
```

**Message Loss Detection**:
```bash
# Check for sequence gaps
ros2 tool echo /competition/telemetry_data | \
  python3 -c "
import sys, json, time
prev_time = None
for line in sys.stdin:
    if line.startswith('{'):
        msg = json.loads(line.replace(\"data: '\", '').rstrip(\"'\\n\"))
        curr_time = msg['gps_saati']['saat'] * 3600 + \
                    msg['gps_saati']['dakika'] * 60 + \
                    msg['gps_saati']['saniye'] + \
                    msg['gps_saati']['milisaniye']/1000
        if prev_time and abs(curr_time - prev_time) > 1.1:
            print(f'Gap detected: {curr_time - prev_time}s')
        prev_time = curr_time
"
```

---

## 📁 Implementation Details

**Aggregator Entry Point**: [telemetry_aggregator_node.py](server_node/telemetry_aggregator_node.py)
- Callbacks registered in `__init__`
- Timer fires at 1 Hz → calls `publish_telemetry()`
- Subscription QoS: `rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, depth=10)`

**Bridge Entry Point**: [bridge.py](server_node/bridge.py)
- Initializes HTTP session with `requests.Session()`
- Creates three timers: login, telemetry send, fetch cycle
- Error recovery: Maintains state across failed requests

**Setup Configuration**: [setup.py](setup.py)
- Entry points:
  - `telemetry_aggregator = server_node.telemetry_aggregator_node:main`
  - `bridge = server_node.bridge:main`

### Sensor Fusion Algorithm

The aggregator implements a simple weighted average for velocity:

```python
# Ground velocity from PX4 local frame (NED convention)
v_x, v_y, v_z = msg.vx, msg.vy, msg.vz
v_ground = sqrt(v_x² + v_y²)  # Horizontal components only

# Selection logic for final velocity
if v_ground > 0.5:
    velocity = v_ground
elif airspeed_reading > 0:
    velocity = airspeed_reading  # Fallback for low-speed hovering
else:
    velocity = v_ground  # Default to ground speed
```

---

## 📋 ROS2 Configuration Reference

### QoS Settings

Both nodes use `BEST_EFFORT` reliability (fire-and-forget) to minimize latency:
- Duration history: 10 messages in buffer
- Lifespan: infinite (no stale message dropping)  
- Deadline: not enforced

### Node Names

- Aggregator: `/telemetry_aggregator`
- Bridge: `/competition_bridge`

### Namespace

All competition topics use namespace `/competition/` for isolation

---

## 🔧 Server Integration API

### Authentication

**Endpoint**: `POST /api/giris`
```json
{
  "kadi": "zarqa",
  "sifre": "1234"
}
```

Launches once on bridge startup; token not subsequently used (current implementation).

### Telemetry Transmission

**Endpoint**: `POST /api/telemetri_gonder`  
**Frequency**: ~0.6 Hz (1.666s interval)  
**Timeout**: 5 seconds  
**Header**: `X-Kadi: zarqa`

Request schema: Full telemetry JSON from aggregator  
Response schema: 
```json
{
  "sunucusaati": {...},
  "konumBilgileri": [
    {"team_id": 2, "lat": 41.5, "lon": 36.1, ...},
    {"team_id": 3, "lat": 41.6, "lon": 36.2, ...}
  ]
}
```

### Data Retrieval

All use `GET` with `X-Kadi` header; fire in parallel every 5 seconds:
- `/api/qr_koordinati` → QR bounding boxes
- `/api/hss_koordinatlari` → HSS no-fly polygon coordinates
- `/api/sunucusaati` → Server timestamp

### Mission Commands

Bridge publishes received commands to:
- `/mission/lockdown_command` → `POST /api/kilitlenme_bilgisi`
- `/mission/kamikaze_command` → `POST /api/kamikaze_bilgisi`

---

## 📖 Technical References

- [ROS2 Humble API](https://docs.ros.org/en/humble/api/)
- [PX4 UxRCE DDS](https://docs.px4.io/main/en/middleware/uxrce_dds.html)
- [px4_msgs Repository](https://github.com/PX4/px4_msgs/tree/humble/src/msg)
- [Python json Module](https://docs.python.org/3/library/json.html)
- [Python requests Library](https://requests.readthedocs.io/)

### Additional Resources

- [Euler Angles Convention](https://en.wikipedia.org/wiki/Euler_angles)
- [Quaternion Mathematics](https://math.stackexchange.com/questions/1031482/euler-angle-to-quaternion-then-quaternion-to-euler-angle)

---

## 📝 License

Apache License 2.0 - See LICENSE file for details