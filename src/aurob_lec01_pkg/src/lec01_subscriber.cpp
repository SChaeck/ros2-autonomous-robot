#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using std::placeholders::_1;

class lec01_subscriber : public rclcpp::Node
{
public:
    lec01_subscriber() : Node("lec01_subscriber")
    {
        lec01_subscriber_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "topic_lec01_aaa", 10, std::bind(&lec01_subscriber::lec01_topic_callback, this, _1));
    }

private:
    void lec01_topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) const
    {
        if (msg->data.size() >= 3)
        {
            int num1 = msg->data[0];
            int num2 = msg->data[1];
            int count = msg->data[2];

            RCLCPP_INFO(this->get_logger(), "Received: num1*count=%d, num2*count=%d", num1 * count, num2 * count);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received message with insufficient data.");
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr lec01_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lec01_subscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}