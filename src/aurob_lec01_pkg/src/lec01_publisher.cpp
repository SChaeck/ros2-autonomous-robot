#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

class lec01_publisher : public rclcpp::Node
{
public:
    lec01_publisher(int num1, int num2) : Node("lec01_publisher"), count_(0)
    {
        _num1 = num1;
        _num2 = num2;
        lec01_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("topic_lec01_aaa", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&lec01_publisher::lec01_timer_callback, this));
    }

private:
    int _num1, _num2;
    int sum;
    void lec01_timer_callback()
    {
        sum = _num1 + _num2;

        auto message_lec01 = std_msgs::msg::Int32MultiArray();
        message_lec01.data = {_num1, _num2, static_cast<int>(count_++)};

        RCLCPP_INFO(this->get_logger(), "Publishing: [%d, %d, %d]",
                    message_lec01.data[0],
                    message_lec01.data[1],
                    message_lec01.data[2]);

        lec01_publisher_->publish(message_lec01);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr lec01_publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    int num1, num2;
    scanf("%d %d", &num1, &num2);
    rclcpp::init(argc, argv);
    auto aa = std::make_shared<lec01_publisher>(num1, num2);
    rclcpp::spin(aa);
    rclcpp::shutdown();
    return 0;
}