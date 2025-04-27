#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/srv/lec02_srv.hpp"

#include <memory>
#include <ctime>

void add(const std::shared_ptr<more_interfaces::srv::Lec02Srv::Request> request,
         std::shared_ptr<more_interfaces::srv::Lec02Srv::Response> response,
         int n)
{
    auto now = std::chrono::steady_clock::now();
    double cur_time = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();

    response->result = request->a * request->b + n;

    RCLCPP_INFO(rclcpp::get_logger(" rclcpp "), "%d * %d + %d = %d",
                request->a, request->b, n, response->result);

    response->time_interval = cur_time - request->time;

    RCLCPP_INFO(rclcpp::get_logger(" rclcpp "), "Time interval about input to processing: %f", response->time_interval);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    int n;

    printf("Please enter one integer.\n=> ");

    scanf("%d", &n);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("srvcli_lec02_server");

    std::function<void(const std::shared_ptr<more_interfaces::srv::Lec02Srv::Request>,
                       std::shared_ptr<more_interfaces::srv::Lec02Srv::Response>)>
        bound_callback = std::bind(&add, std::placeholders::_1, std::placeholders::_2, n);
    rclcpp::Service<more_interfaces::srv::Lec02Srv>::SharedPtr service = node->create_service<more_interfaces::srv::Lec02Srv>("srvcli_lec02", bound_callback);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to request.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}