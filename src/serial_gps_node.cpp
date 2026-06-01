#include "ros/ros.h"
#include <serial/serial.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <nav_msgs/Odometry.h>

// Must match the Arduino-side struct layout exactly
struct __attribute__((packed)) GPS_Data
{
    uint32_t timestamp;   // 時間戳 (微秒)
    int32_t latitude;     // 緯度 (deg * 1e7)
    int32_t longitude;    // 經度 (deg * 1e7)
    int32_t altitude;     // 高度 (mm)
    uint8_t GPSFixType;   // GPS 定位類型
    uint8_t RTKStatus;    // RTK 狀態
};

// Must match the Arduino-side SLAM_position struct layout exactly
struct __attribute__((packed)) SLAM_position
{
    uint64_t timestamp;
    float pos[3];
    float orientation[4];
};

static const uint8_t START_FRAME[2] = {0xBB, 0xAA};

// Global serial pointer for use in callback
static serial::Serial *g_ser = nullptr;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (!g_ser || !g_ser->isOpen())
        return;

    SLAM_position slam;
    slam.timestamp = static_cast<uint64_t>(ros::Time::now().toNSec() / 1000); // 用當下發送時間，轉成微秒
    slam.pos[0] = static_cast<float>(msg->pose.pose.position.x);
    slam.pos[1] = static_cast<float>(msg->pose.pose.position.y);
    slam.pos[2] = static_cast<float>(msg->pose.pose.position.z);
    slam.orientation[0] = static_cast<float>(msg->pose.pose.orientation.x);
    slam.orientation[1] = static_cast<float>(msg->pose.pose.orientation.y);
    slam.orientation[2] = static_cast<float>(msg->pose.pose.orientation.z);
    slam.orientation[3] = static_cast<float>(msg->pose.pose.orientation.w);

    g_ser->write(START_FRAME, 2);
    g_ser->write(reinterpret_cast<const uint8_t *>(&slam), sizeof(SLAM_position));

    ROS_INFO("SLAM sent | ts=%lu | pos=[%.3f, %.3f, %.3f] | ori=[%.3f, %.3f, %.3f, %.3f]",
             (unsigned long)slam.timestamp,
             slam.pos[0], slam.pos[1], slam.pos[2],
             slam.orientation[0], slam.orientation[1], slam.orientation[2], slam.orientation[3]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_gps_node");
    ros::NodeHandle nh("~");

    // Load parameters
    std::string port;
    int baud_rate, timeout_ms;
    std::string frame_id, topic;

    nh.param<std::string>("serial/port", port, "/dev/ttyACM0");
    nh.param<int>("serial/baud_rate", baud_rate, 115200);
    nh.param<int>("serial/timeout_ms", timeout_ms, 1000);
    nh.param<std::string>("gps/frame_id", frame_id, "gps_link");
    nh.param<std::string>("gps/topic", topic, "gps/fix");
    std::string navsat_topic;
    nh.param<std::string>("gps/navsat_topic", navsat_topic, "gps/navsat_fix");

    std::string odom_topic;
    nh.param<std::string>("slam/odom_topic", odom_topic, "/odom");

    ros::NodeHandle nh_global;
    ros::Publisher gps_pub = nh_global.advertise<std_msgs::Float64MultiArray>(topic, 10);
    ros::Publisher navsat_pub = nh_global.advertise<sensor_msgs::NavSatFix>(navsat_topic, 10);
    ros::Subscriber odom_sub = nh_global.subscribe(odom_topic, 10, odomCallback);

    // Setup serial
    serial::Serial ser;
    ser.setPort(port);
    ser.setBaudrate(baud_rate);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms);
    ser.setTimeout(timeout);

    try {
        ser.open();
    } catch (serial::IOException &e) {
        ROS_ERROR("Unable to open port %s: %s", port.c_str(), e.what());
        return -1;
    }

    if (!ser.isOpen()) {
        ROS_ERROR("Serial port %s not open", port.c_str());
        return -1;
    }

    ROS_INFO("Serial port %s opened (baud: %d)", port.c_str(), baud_rate);

    g_ser = &ser;

    // State machine for binary protocol: find 0xBB 0xAA, then read sizeof(GPS_Data)
    enum State { WAIT_HEADER_0, WAIT_HEADER_1, READ_PAYLOAD };
    State state = WAIT_HEADER_0;
    uint8_t payload_buf[sizeof(GPS_Data)];
    size_t payload_count = 0;

    while (ros::ok()) {
        if (ser.available()) {
            std::string raw = ser.read(ser.available());

            for (size_t i = 0; i < raw.size(); ++i) {
                uint8_t b = static_cast<uint8_t>(raw[i]);

                switch (state) {
                case WAIT_HEADER_0:
                    if (b == START_FRAME[0])
                        state = WAIT_HEADER_1;
                    break;

                case WAIT_HEADER_1:
                    if (b == START_FRAME[1]) {
                        state = READ_PAYLOAD;
                        payload_count = 0;
                    } else {
                        state = WAIT_HEADER_0;
                    }
                    break;

                case READ_PAYLOAD:
                    payload_buf[payload_count++] = b;
                    if (payload_count == sizeof(GPS_Data)) {
                        GPS_Data gps;
                        memcpy(&gps, payload_buf, sizeof(GPS_Data));

                        // data[0]=timestamp, data[1]=lat, data[2]=lon,
                        // data[3]=alt(m), data[4]=GPSFixType, data[5]=RTKStatus
                        std_msgs::Float64MultiArray msg;
                        msg.data.resize(6);
                        msg.data[0] = static_cast<double>(gps.timestamp);
                        msg.data[1] = gps.latitude / 1e7;
                        msg.data[2] = gps.longitude / 1e7;
                        msg.data[3] = gps.altitude / 1000.0;
                        msg.data[4] = static_cast<double>(gps.GPSFixType);
                        msg.data[5] = static_cast<double>(gps.RTKStatus);

                        gps_pub.publish(msg);

                        // Publish standard NavSatFix message
                        sensor_msgs::NavSatFix navsat_msg;
                        navsat_msg.header.stamp = ros::Time::now();
                        navsat_msg.header.frame_id = frame_id;

                        // Map GPSFixType to NavSatStatus
                        navsat_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
                        if (gps.GPSFixType == 0) {
                            navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
                        } else if (gps.RTKStatus == 2) {
                            navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX; // RTK fixed
                        } else if (gps.RTKStatus == 1) {
                            navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX; // RTK float
                        } else {
                            navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
                        }

                        navsat_msg.latitude = gps.latitude / 1e7;
                        navsat_msg.longitude = gps.longitude / 1e7;
                        navsat_msg.altitude = gps.altitude / 1000.0;

                        navsat_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

                        navsat_pub.publish(navsat_msg);

                        ROS_INFO("GPS | ts=%u | lat=%.7f lon=%.7f alt=%.3f | fix=%u rtk=%u",
                                 gps.timestamp, gps.latitude / 1e7, gps.longitude / 1e7, gps.altitude / 1000.0,
                                 gps.GPSFixType, gps.RTKStatus);

                        state = WAIT_HEADER_0;
                    }
                    break;
                }
            }
        }

        ros::spinOnce();
    }

    ser.close();
    return 0;
}
