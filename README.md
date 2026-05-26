# serial_to_ros

ROS Noetic package for reading GPS and IMU data from serial ports and publishing as standard ROS messages. Supports bidirectional communication — receives sensor data from Arduino and sends SLAM odometry back.

## Architecture

The package contains two independent nodes:

- **serial_gps_node** — reads binary GPS packets from serial, publishes `sensor_msgs/NavSatFix` and `std_msgs/Float64MultiArray`, and writes SLAM position back to Arduino.
- **serial_imu_node** — reads ASCII IMU data from serial, publishes `sensor_msgs/Imu`.

## Serial Data Formats

### GPS Node — Binary Protocol

GPS node uses a framed binary protocol with packed structs. The same serial port handles both receiving GPS data and sending SLAM data.

**Frame structure:**

```
[0xBB] [0xAA] [payload bytes...]
```

No length field or checksum — the payload size is determined by the struct type.

#### GPS Data (Arduino → ROS), 18 bytes payload

```c
struct __attribute__((packed)) GPS_Data {
    uint32_t timestamp;   // microseconds
    int32_t  latitude;    // degrees × 1e7 (e.g. 221234567 = 22.1234567°)
    int32_t  longitude;   // degrees × 1e7 (e.g. 1201234567 = 120.1234567°)
    int32_t  altitude;    // millimeters (divide by 1000.0 for meters)
    uint8_t  GPSFixType;  // 0 = no fix
    uint8_t  RTKStatus;   // 0 = none, 1 = float, 2 = fixed
};
```

| Field | Type | Unit | Conversion |
|---|---|---|---|
| `timestamp` | `uint32_t` | microseconds | — |
| `latitude` | `int32_t` | degrees × 10^7 | `value / 1e7` → degrees |
| `longitude` | `int32_t` | degrees × 10^7 | `value / 1e7` → degrees |
| `altitude` | `int32_t` | millimeters | `value / 1000.0` → meters |
| `GPSFixType` | `uint8_t` | — | 0 = no fix |
| `RTKStatus` | `uint8_t` | — | 0 = none, 1 = float, 2 = fixed |

#### SLAM Position (ROS → Arduino), 16 bytes payload

```c
struct __attribute__((packed)) SLAM_position {
    uint32_t timestamp;   // microseconds (from ROS header stamp)
    float    x;
    float    y;
    float    z;
};
```

When the GPS node receives odometry messages (default topic: `/Odometry`), it packs the position into this struct and sends it over serial with the `0xBB 0xAA` header.

#### GPS Published Topics

**`gps/fix`** (`std_msgs/Float64MultiArray`):

| Index | Value |
|---|---|
| `data[0]` | timestamp (microseconds) |
| `data[1]` | latitude (degrees) |
| `data[2]` | longitude (degrees) |
| `data[3]` | altitude (meters) |
| `data[4]` | GPSFixType |
| `data[5]` | RTKStatus |

**`gps/navsat_fix`** (`sensor_msgs/NavSatFix`):

- `header.frame_id` = `gps_link`
- `latitude`, `longitude` in degrees, `altitude` in meters
- RTK status mapping:
  - `GPSFixType == 0` → `STATUS_NO_FIX`
  - `RTKStatus == 2` (fixed) → `STATUS_GBAS_FIX`
  - `RTKStatus == 1` (float) → `STATUS_SBAS_FIX`
  - otherwise → `STATUS_FIX`

### IMU Node — ASCII Protocol

IMU node reads newline-terminated ASCII text, one sample per line:

```
timestamp ax ay az gx gy gz\n
```

| Field | Unit | Notes |
|---|---|---|
| `timestamp` | — | Device timestamp (logged only, not used in ROS message) |
| `ax ay az` | g | Linear acceleration, multiplied by `gravity` param to get m/s² |
| `gx gy gz` | rad/s | Angular velocity, published directly |

#### IMU Published Topic

**`imu/data_raw`** (`sensor_msgs/Imu`):

- `header.frame_id` = `imu_link`
- `linear_acceleration` = acceleration × gravity (m/s²)
- `angular_velocity` in rad/s
- `orientation_covariance[0]` = -1 (orientation not available)

## Dependencies

- ROS Noetic
- `roscpp`
- `sensor_msgs`
- [`serial`](http://wiki.ros.org/serial) — ROS serial library

Install serial:

```bash
sudo apt-get install ros-noetic-serial
```

## Build

```bash
cd <workspace>
catkin build
source devel/setup.bash
```

## Configuration

### GPS Node — `config/serial_gps.yaml`

```yaml
serial:
  port: "/dev/ttyUSB0"
  baud_rate: 115200
  timeout_ms: 1000

gps:
  frame_id: "gps_link"
  topic: "gps/fix"

slam:
  odom_topic: "/Odometry"
```

| Parameter | Description | Default |
|---|---|---|
| `serial/port` | Serial port path | `/dev/ttyACM0` |
| `serial/baud_rate` | Baud rate | `115200` |
| `serial/timeout_ms` | Read timeout (ms) | `1000` |
| `gps/frame_id` | TF frame ID | `gps_link` |
| `gps/topic` | Float64MultiArray topic | `gps/fix` |
| `gps/navsat_topic` | NavSatFix topic | `gps/navsat_fix` |
| `slam/odom_topic` | Odometry subscription topic | `/odom` |

### IMU Node — `config/serial_imu.yaml`

```yaml
serial:
  port: "/dev/ttyACM0"
  baud_rate: 115200
  timeout_ms: 1000

imu:
  frame_id: "imu_link"
  topic: "imu/data_raw"
  gravity: 9.81
```

| Parameter | Description | Default |
|---|---|---|
| `serial/port` | Serial port path | `/dev/ttyACM0` |
| `serial/baud_rate` | Baud rate | `115200` |
| `serial/timeout_ms` | Read timeout (ms) | `1000` |
| `imu/frame_id` | TF frame ID | `imu_link` |
| `imu/topic` | Published topic name | `imu/data_raw` |
| `imu/gravity` | Gravity constant (g → m/s²) | `9.81` |

## Usage

Launch GPS node:

```bash
roslaunch serial_to_ros serial_gps.launch
```

Launch IMU node:

```bash
roslaunch serial_to_ros serial_imu.launch
```

Verify output:

```bash
rostopic echo /gps/navsat_fix
rostopic echo /gps/fix
rostopic echo /imu/data_raw
```

## Arduino Reference

The repository includes `main.cpp` as an Arduino-side reference implementation. It demonstrates:

- GPS data transmission at 5 Hz via `Serial1` (binary protocol)
- SLAM position reception via `Serial1` (binary protocol)
- Debug output at 10 Hz via `Serial` (USB, ASCII)
