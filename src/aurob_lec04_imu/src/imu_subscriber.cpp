#include <memory>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ImuSubscriber : public rclcpp::Node
{
public:
    ImuSubscriber() : Node("imu_subscriber")
    {
        RCLCPP_INFO(this->get_logger(), "IMU Subscriber Node Started");

        auto default_qos = rclcpp::SensorDataQoS();

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&ImuSubscriber::imu_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "IMU Subscription Set Up (Topic: /imu/data)");

        lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", default_qos,
            std::bind(&ImuSubscriber::lidar_callback, this, _1));

        timer_ = this->create_wall_timer(
            500ms, std::bind(&ImuSubscriber::timer_callback, this));
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // IMU 데이터 처리
        // RCLCPP_INFO(this->get_logger(), "IMU Data Received: Orientation (x: %.2f, y: %.2f, z: %.2f)",
        //             msg->orientation.x, msg->orientation.y, msg->orientation.z);
        // RCLCPP_INFO(this->get_logger(), "Angular Velocity (x: %.2f, y: %.2f, z: %.2f)",
        //             msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        // RCLCPP_INFO(this->get_logger(), "Linear Acceleration (x: %.2f, y: %.2f, z: %.2f)",
        //             msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

        // 쿼터니언 값을 tf2::Quaternion으로 변환
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        // 오일러 각도로 변환 (라디안 단위)
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 라디안을 도(degree)로 변환
        // double roll_deg = roll * 180.0 / M_PI;
        double pitch_deg = pitch * 180.0 / M_PI;
        // double yaw_deg = yaw * 180.0 / M_PI;

        // RCLCPP_INFO(this->get_logger(), "IMU Data Received: Roll: %.2f, Pitch: %.2f, Yaw: %.2f degrees", roll_deg, pitch_deg, yaw_deg);

        if (!front_30_degree && pitch_deg > 30.0)
        {
            front_30_degree = true;
            // RCLCPP_INFO(this->get_logger(), "Front 30 Degree Detected");
        }
        else if (!back_30_degree && pitch_deg < -30.0)
        {
            back_30_degree = true;
            // RCLCPP_INFO(this->get_logger(), "Back 30 Degree Detected");
        }
        else
        {
            front_30_degree = false;
            back_30_degree = false;
        }
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {
        latest_scan_ = *_msg;
    }

    void timer_callback()
    {
        int index;
        if (front_30_degree)
        {
            RCLCPP_INFO(this->get_logger(), "Front 30 Degree Lidar Data Received");
            index = calculate_index_from_degree(0, latest_scan_.ranges.size());
            float distance = latest_scan_.ranges[index];
            RCLCPP_INFO(this->get_logger(), "Distance at 0 Degree: %.2f cm", distance * 100);
        }
        else if (back_30_degree)
        {
            RCLCPP_INFO(this->get_logger(), "Back 30 Degree Lidar Data Received");
            index = calculate_index_from_degree(180, latest_scan_.ranges.size());
            float distance = latest_scan_.ranges[index];
            RCLCPP_INFO(this->get_logger(), "Distance at 180 Degree: %.2f cm", distance * 100);
        }
    }

    int calculate_index_from_degree(int degree, size_t range_size)
    {
        return static_cast<int>(degree * range_size / 360.0);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_;
    sensor_msgs::msg::LaserScan latest_scan_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool front_30_degree = false;
    bool back_30_degree = false;
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto imu_subscriber = std::make_shared<ImuSubscriber>();
    rclcpp::spin(imu_subscriber);
    rclcpp::shutdown();
    return 0;
}