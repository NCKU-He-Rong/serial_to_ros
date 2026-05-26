#include "ros/ros.h"
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sstream>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_to_ros_node");
    ros::NodeHandle nh("~");

    // Load parameters from yaml
    std::string port;
    int baud_rate, timeout_ms;
    std::string frame_id, topic;

    nh.param<std::string>("serial/port", port, "/dev/ttyACM0");
    nh.param<int>("serial/baud_rate", baud_rate, 115200);
    nh.param<int>("serial/timeout_ms", timeout_ms, 1000);
    nh.param<std::string>("imu/frame_id", frame_id, "imu_link");
    nh.param<std::string>("imu/topic", topic, "imu/data_raw");
    double gravity;
    nh.param<double>("imu/gravity", gravity, 9.81);
    // Use global NodeHandle for topic publishing
    ros::NodeHandle nh_global;
    ros::Publisher imu_pub = nh_global.advertise<sensor_msgs::Imu>(topic, 10);

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

    std::string buffer;

    while (ros::ok()) {
        if (ser.available()) {
            buffer += ser.read(ser.available());

            size_t pos;
            while ((pos = buffer.find('\n')) != std::string::npos) {
                std::string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);

                if (!line.empty() && line.back() == '\r')
                    line.pop_back();

                if (line.empty())
                    continue;

                std::istringstream iss(line);
                double timestamp, ax, ay, az, gx, gy, gz;
                if (iss >> timestamp >> ax >> ay >> az >> gx >> gy >> gz) {
                    ROS_INFO("t=%.3f | acc=(%.4f, %.4f, %.4f) | gyro=(%.4f, %.4f, %.4f)",
                             timestamp, ax, ay, az, gx, gy, gz);

                    sensor_msgs::Imu imu_msg;
                    imu_msg.header.stamp = ros::Time::now();
                    imu_msg.header.frame_id = frame_id;

                    imu_msg.linear_acceleration.x = ax * gravity;
                    imu_msg.linear_acceleration.y = ay * gravity;
                    imu_msg.linear_acceleration.z = az * gravity;

                    imu_msg.angular_velocity.x = gx;
                    imu_msg.angular_velocity.y = gy;
                    imu_msg.angular_velocity.z = gz;

                    imu_msg.orientation_covariance[0] = -1;

                    imu_pub.publish(imu_msg);
                } else {
                    ROS_WARN("Parse failed: %s", line.c_str());
                }
            }
        }

        ros::spinOnce();
    }

    ser.close();
    return 0;
}
