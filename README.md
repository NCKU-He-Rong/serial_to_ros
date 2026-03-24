# serial_to_ros

ROS Noetic node for reading IMU data from a serial port and publishing as `sensor_msgs/Imu`.

## Serial Data Format

The node expects ASCII data from the serial device, one sample per line:

```
timestamp ax ay az gx gy gz\n
```

- Fields separated by spaces
- `timestamp` — device timestamp
- `ax ay az` — linear acceleration
- `gx gy gz` — angular velocity

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

Edit `config/serial_imu.yaml`:

```yaml
serial:
  port: "/dev/ttyACM0"
  baud_rate: 115200
  timeout_ms: 1000

imu:
  frame_id: "imu_link"
  topic: "imu/data_raw"
```

| Parameter | Description | Default |
|---|---|---|
| `serial/port` | Serial port path | `/dev/ttyACM0` |
| `serial/baud_rate` | Baud rate | `115200` |
| `serial/timeout_ms` | Read timeout (ms) | `1000` |
| `imu/frame_id` | TF frame ID | `imu_link` |
| `imu/topic` | Published topic name | `imu/data_raw` |

## Usage

```bash
roslaunch serial_to_ros serial_imu.launch
```

Verify output:

```bash
rostopic echo /imu/data_raw
```
