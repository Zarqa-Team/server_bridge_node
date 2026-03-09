# Telemetry Data Reference

## Overview

The Competition Bridge Node aggregates dynamic telemetry data from PX4 SITL topics and sends it to the competition server. This document describes the telemetry data structure, field mappings, and conversion details.

## Telemetry Data Structure

```json
{
  "takim_numarasi": 1,           // Team number (fixed)
  "iha_enlem": 0.0,             // Latitude (degrees, -90 to 90)
  "iha_boylam": 0.0,            // Longitude (degrees, -180 to 180)
  "iha_irtifa": 0.0,            // Altitude above sea level (meters)
  "iha_dikilme": 0.0,           // Roll angle (degrees, -180 to 180)
  "iha_yonelme": 0.0,           // Yaw angle (degrees, -180 to 180)
  "iha_yatis": 0.0,             // Pitch angle (degrees, -180 to 180)
  "iha_hiz": 0.0,               // Velocity magnitude (m/s, ground speed or airspeed)
  "iha_batarya": 0.0,           // Battery remaining (percentage, 0-100)
  "iha_otonom": 0,              // Autonomous mode (0=manual, 1=autonomous)
  "iha_kilitlenme": 0,          // Target lock status (0=not locked, 1=locked)
  "hedef_merkez_X": 0,          // Target center X coordinate
  "hedef_merkez_Y": 0,          // Target center Y coordinate
  "hedef_genislik": 0,          // Target width
  "hedef_yukseklik": 0,         // Target height
  "gps_saati": {                // UTC time extracted from GPS timestamp
    "saat": 0,                  // Hours (0-23)
    "dakika": 0,                // Minutes (0-59)
    "saniye": 0,                // Seconds (0-59)
    "milisaniye": 0             // Milliseconds (0-999)
  }
}
```

## Field Mappings from PX4 Topics

### Position and Navigation

| Field | PX4 Topic | Source Field | Conversion | Validation |
|-------|-----------|-------------|-----------|-----------|
| `iha_enlem` | `/fmu/out/vehicle_global_position` | `lat` | `msg.lat / 1e7` | -90 to 90 degrees |
| `iha_boylam` | `/fmu/out/vehicle_global_position` | `lon` | `msg.lon / 1e7` | -180 to 180 degrees |
| `iha_irtifa` | `/fmu/out/vehicle_global_position` | `alt` | `msg.alt / 1000.0` (mm to m) | ≥ 0 meters |
| `gps_saati` | `/fmu/out/vehicle_global_position` | `timestamp` | Extract hours/min/sec/ms | 0-23, 0-59, 0-59, 0-999 |

### Attitude (Orientation)

| Field | PX4 Topic | Source Field | Conversion | Validation |
|-------|-----------|-------------|-----------|-----------|
| `iha_dikilme` (Roll) | `/fmu/out/vehicle_attitude` | `q[x,y,z,w]` | Quaternion → Euler (X-axis) | -180 to 180 degrees |
| `iha_yonelme` (Yaw) | `/fmu/out/vehicle_attitude` | `q[x,y,z,w]` | Quaternion → Euler (Z-axis) | -180 to 180 degrees |
| `iha_yatis` (Pitch) | `/fmu/out/vehicle_attitude` | `q[x,y,z,w]` | Quaternion → Euler (Y-axis) | -180 to 180 degrees |

### Velocity

| Field | PX4 Topic | Source Field | Conversion | Validation |
|-------|-----------|-------------|-----------|-----------|
| `iha_hiz` | `/fmu/out/vehicle_local_position` | `vx, vy, vz` | `sqrt(vx² + vy² + vz²)` (m/s) | ≥ 0 |
| `iha_hiz` | `/fmu/out/airspeed_validated` | `true_airspeed_m_s` | Direct (overrides ground speed) | ≥ 0 |

**Note**: True airspeed (from `airspeed_validated`) overrides ground speed when available, as it's more accurate in windy conditions.

### Power

| Field | PX4 Topic | Source Field | Conversion | Validation |
|-------|-----------|-------------|-----------|-----------|
| `iha_batarya` | `/fmu/out/battery_status` | `remaining` | Direct (percentage) | 0 to 100 |

### Vehicle Status

| Field | PX4 Topic | Source Field | Conversion | Validation |
|-------|-----------|-------------|-----------|-----------|
| `iha_otonom` | `/fmu/out/vehicle_status` | `nav_state` | `1 if nav_state in [3, 14] else 0` | 0 or 1 |
| `iha_kilitlenme` | Lockdown Command | `is_locked` | Set by `update_lockdown_target()` | 0 or 1 |

**Autonomous Mode Values**:
- `nav_state = 3`: AUTO_MISSION
- `nav_state = 14`: OFFBOARD
- Other values: Manual or other modes

### Target Lock and Coordinates

**Mapping**: Controlled via lockdown commands through `/mission/lockdown_command` topic

| Field | Source | Set By | Description |
|-------|--------|--------|-------------|
| `iha_kilitlenme` | Lockdown command | `update_lockdown_target()` | 1 when locked, 0 when not |
| `hedef_merkez_X` | Lockdown command | `update_lockdown_target()` | Target center X (pixels or normalized) |
| `hedef_merkez_Y` | Lockdown command | `update_lockdown_target()` | Target center Y (pixels or normalized) |
| `hedef_genislik` | Lockdown command | `update_lockdown_target()` | Bounding box width |
| `hedef_yukseklik` | Lockdown command | `update_lockdown_target()` | Bounding box height |

## Quaternion to Euler Conversion

The node converts PX4 quaternions (w, x, y, z) to Euler angles using the standard formula:

```
Roll  = atan2(2(wz + xy), 1 - 2(x² + y²))
Pitch = asin(2(wy - zx))
Yaw   = atan2(2(wz + xy), 1 - 2(y² + z²))
```

All angles are converted to degrees and constrained to [-180, 180] range.

## GPS Time Extraction

The PX4 timestamp (in microseconds since Unix epoch: 1970-01-01 00:00:00 UTC) is extracted to provide wall-clock time:

```
total_seconds = gps_time_us / 1,000,000
hours = (total_seconds // 3600) % 24        // [0-23]
minutes = (total_seconds // 60) % 60        // [0-59]
seconds = total_seconds % 60                // [0-59]
milliseconds = (gps_time_us % 1,000,000) / 1000  // [0-999]
```

## Data Publishing

### To Competition Server
- **Topic**: `/api/telemetri_gonder` (HTTP POST)
- **Frequency**: 600 ms (0.6 seconds)
- **Message Format**: Complete `telemetry_data` dictionary as JSON

### To ROS System
- **Topic**: `/competition/telemetry_data`
- **Message Type**: `std_msgs/String` (JSON encoded)
- **Frequency**: 1.0 second

## Known Limitations

1. **Ground Speed vs Airspeed**: Wind conditions and low-speed flight may cause ground speed and airspeed to diverge significantly.
2. **GPS Time**: Only wall-clock time (hours/min/sec) is extracted; date information is not included.
3. **Altitude Reference**: Altitude is relative to sea level and may not be accurate in areas with significant local variation.

## Data Quality Assurance

The node implements the following checks:

- **Value Range Validation**: All angles, coordinates, and percentages are validated to be within expected ranges
- **Zero Initialization**: All telemetry fields start at zero and update asynchronously
- **Error Logging**: Any conversion or processing errors are logged with error level

## PX4 Topic Subscriptions

```python
'/fmu/out/vehicle_attitude'         → VehicleAttitude
'/fmu/out/vehicle_local_position'   → VehicleLocalPosition
'/fmu/out/vehicle_global_position'  → VehicleGlobalPosition
'/fmu/out/battery_status'           → BatteryStatus
'/fmu/out/vehicle_status'           → VehicleStatus
'/fmu/out/airspeed_validated'       → AirspeedValidated
```

All subscriptions use BEST_EFFORT QoS profile for compatibility with sensor data streams.

## Debug Information

Enable debug logging to monitor data processing:

```bash
ros2 run server_node server_node --ros-args --log-level debug
```

This will print detailed information about:
- Each telemetry update
- Sensor data values before and after conversion
- Time synchronization
- Server communication status
