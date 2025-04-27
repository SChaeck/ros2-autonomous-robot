#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>

/* package dependencies --> CMakelist.txt, package.xml*/

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/lec02_msg.hpp"

using namespace std::chrono_literals;

class lec02_publisher : public rclcpp::Node
{
public:
    lec02_publisher() : Node("lec02_publisher"), count_(0)
    {
        index = 0;
        lec02_publisher_ = this->create_publisher<more_interfaces::msg::Lec02Msg>("topic_lec02_aaa", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&lec02_publisher::lec02_timer_callback, this));
    }

private:
    void lec02_timer_callback()
    {
        scanf("%d %d", &num1, &num2);
        auto now = std::chrono::steady_clock::now();
        double cur_time = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();

        index++;

        auto message_lec02 = more_interfaces::msg::Lec02Msg();
        message_lec02.a = num1;
        message_lec02.b = num2;
        message_lec02.time = cur_time;
        message_lec02.index = index;

        RCLCPP_INFO(this->get_logger(), "a: %d\nb: %d\ntime: %f\nindex: %d\n\n", message_lec02.a, message_lec02.b, message_lec02.time, message_lec02.index);

        lec02_publisher_->publish(message_lec02);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<more_interfaces::msg::Lec02Msg>::SharedPtr lec02_publisher_;
    size_t count_;

    int num1, num2, index;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto aa = std::make_shared<lec02_publisher>();
    rclcpp::spin(aa);
    rclcpp::shutdown();
    return 0;
}
