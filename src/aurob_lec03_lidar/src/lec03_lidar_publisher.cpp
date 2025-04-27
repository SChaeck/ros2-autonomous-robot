#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

class lec03_lidar_publisher : public rclcpp::Node
{
public:
    lec03_lidar_publisher(int num1, int num2, int num3) : Node("lec03_lidar_publisher"), count_(0)
    {
        _num1 = num1;
        _num2 = num2;
        _num3 = num3;
        lec03_lidar_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("topic_lec03_aaa", 10);

        // 주기적으로 메시지를 발행하기 위한 타이머 추가 (500ms 주기)
        timer_ = this->create_wall_timer(500ms, std::bind(&lec03_lidar_publisher::send_three_degree, this));

        RCLCPP_INFO(this->get_logger(), "퍼블리셔 시작! 각도: [%d, %d, %d]", _num1, _num2, _num3);
    }

private:
    int _num1, _num2, _num3;
    void send_three_degree()
    {
        auto message_lec03 = std_msgs::msg::Int32MultiArray();
        message_lec03.data = {_num1, _num2, _num3};

        RCLCPP_INFO(this->get_logger(), "Publishing: [%d, %d, %d]",
                    message_lec03.data[0],
                    message_lec03.data[1],
                    message_lec03.data[2]);

        lec03_lidar_publisher_->publish(message_lec03);
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr lec03_lidar_publisher_;
    rclcpp::TimerBase::SharedPtr timer_; // 타이머 멤버 변수 추가
    size_t count_;
};

int main(int argc, char *argv[])
{
    int num1, num2, num3;

    printf("Type 3 degree (e.g. 30 180 330): \n");
    scanf("%d %d %d", &num1, &num2, &num3);

    rclcpp::init(argc, argv);
    auto aa = std::make_shared<lec03_lidar_publisher>(num1, num2, num3);
    rclcpp::spin(aa);
    rclcpp::shutdown();
    return 0;
}