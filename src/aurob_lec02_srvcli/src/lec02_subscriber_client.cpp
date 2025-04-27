#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/lec02_msg.hpp"
#include "more_interfaces/srv/lec02_srv.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class lec02_subscriber : public rclcpp::Node
{
public:
    lec02_subscriber() : Node("lec02_subscriber")
    {
        lec02_subscriber_ = this->create_subscription<more_interfaces::msg::Lec02Msg>(
            "topic_lec02_aaa", 10, std::bind(&lec02_subscriber::lec02_topic_callback, this, _1));
        node = rclcpp::Node::make_shared("srvcli_lec02_client");
        client = node->create_client<more_interfaces::srv::Lec02Srv>("srvcli_lec02");
    }

private:
    void lec02_topic_callback(const more_interfaces::msg::Lec02Msg::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "[Index %d] %d + %d = %d\nInputted time: %f", msg->index, msg->a, msg->b, msg->a + msg->b, msg->time);

        auto request = std::make_shared<more_interfaces::srv::Lec02Srv::Request>();
        request->time = msg->time;
        request->a = msg->a;
        request->b = msg->b;

        while (!client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger(" rclcpp "), "Interrupted while waiting for the service. Existing. ");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger(" rclcpp "), "service not available, wating again...");
        } // Server is ready!

        auto result = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger(" rclcpp "), "Server response: %d, %f", result.get()->result, result.get()->time_interval);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger(" rclcpp "), "Failed to call service srvcli_lec02");
        }
    }
    rclcpp::Subscription<more_interfaces::msg::Lec02Msg>::SharedPtr lec02_subscriber_;
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Client<more_interfaces::srv::Lec02Srv>::SharedPtr client;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto a = std::make_shared<lec02_subscriber>();
    rclcpp::spin(a);
    rclcpp::shutdown();
    return 0;
}