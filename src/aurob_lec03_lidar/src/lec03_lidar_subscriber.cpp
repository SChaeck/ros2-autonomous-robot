#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class laser_read : public rclcpp::Node
{
public:
    laser_read() : Node("lidar__")
    {
        RCLCPP_INFO(this->get_logger(), "구독자 노드 시작됨");

        auto default_qos = rclcpp::SensorDataQoS();

        lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", default_qos,
            std::bind(&laser_read::lidar_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "LiDAR 구독 설정 완료 (토픽: /scan)");

        degrees_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "topic_lec03_aaa", 10,
            std::bind(&laser_read::degrees_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "각도 구독 설정 완료 (토픽: topic_lec03_aaa)");

        timer_ = this->create_wall_timer(
            500ms, std::bind(&laser_read::timer_callback, this));

        obstacle_states_ = {false, false, false};

        // 상태 확인용 타이머 추가
        status_timer_ = this->create_wall_timer(
            2000ms, std::bind(&laser_read::check_status, this));
    }

private:
    void check_status()
    {
        if (degrees_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "아직 각도 데이터를 받지 못했습니다. topic_lec03_aaa 토픽 확인 필요");
        }
        else
        {
            // 각도 데이터가 있으면 더 이상 경고 메시지를 표시하지 않도록 수정
            status_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "모든 필수 데이터가 수신되었습니다. 상태 확인 타이머 중지");
        }

        if (latest_scan_.ranges.empty())
        {
            RCLCPP_WARN(this->get_logger(), "아직 LiDAR 데이터를 받지 못했습니다. /scan 토픽 확인 필요");
        }
    }

    void degrees_callback(const std_msgs::msg::Int32MultiArray::SharedPtr _msg)
    {
        // 이미 데이터가 있으면 무시
        if (!degrees_.empty())
        {
            RCLCPP_DEBUG(this->get_logger(), "이미 각도 데이터가 있습니다. 새 데이터 무시");
            return;
        }

        degrees_.clear();
        for (int degree : _msg->data)
        {
            degrees_.push_back(degree);
            RCLCPP_INFO(this->get_logger(), "수신된 각도: %d", degree);
        }

        RCLCPP_INFO(this->get_logger(), "첫 각도 데이터 설정 완료. 이후 데이터는 무시됩니다.");
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {
        if (latest_scan_.ranges.empty())
        {
            RCLCPP_INFO(this->get_logger(), "첫 LiDAR 데이터 수신됨");
        }
        latest_scan_ = *_msg;
    }

    void timer_callback()
    {
        if (latest_scan_.ranges.empty())
        {
            RCLCPP_DEBUG(this->get_logger(), "LiDAR 데이터가 아직 없습니다");
            return;
        }

        if (degrees_.empty())
        {
            RCLCPP_DEBUG(this->get_logger(), "각도 데이터가 아직 없습니다");
            return;
        }

        // 각 지정된 각도에 대해 거리 확인 및 상태 변화 감지
        for (size_t i = 0; i < degrees_.size(); i++)
        {
            int deg = degrees_[i];
            int index = calculateIndexFromDegree(deg, latest_scan_.ranges.size());

            if (index < 0 || index >= static_cast<int>(latest_scan_.ranges.size()))
            {
                RCLCPP_WARN(this->get_logger(), "각도 %d에 대한 유효하지 않은 인덱스: %d (최대: %zu)",
                            deg, index, latest_scan_.ranges.size() - 1);
                continue; // 유효하지 않은 인덱스 건너뛰기
            }

            float distance = latest_scan_.ranges[index];

            // 거리 출력 (미터에서 센티미터로 변환)
            RCLCPP_INFO(this->get_logger(), "각도 %d: %.2f cm", deg, distance * 100);

            // 장애물 상태 변화 감지 (40cm = 0.4m)
            bool is_obstacle_now = (distance < 0.4);

            // 상태가 변화했는지 확인
            if (i < obstacle_states_.size() && is_obstacle_now != obstacle_states_[i])
            {
                if (is_obstacle_now)
                {
                    RCLCPP_INFO(this->get_logger(), "(%d) 상태변화: 40cm 이상 → 40cm 미만", deg);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "(%d) 상태변화: 40cm 미만 → 40cm 이상", deg);
                }

                // 상태 업데이트
                obstacle_states_[i] = is_obstacle_now;
            }
        }
    }

    // 각도를 LiDAR 인덱스로 변환하는 함수
    int calculateIndexFromDegree(int degree, size_t range_size)
    {
        // 각도 정규화 (0~359)
        while (degree < 0)
            degree += 360;
        degree = degree % 360;

        // 각도를 인덱스로 변환
        return static_cast<int>(degree * range_size / 360.0);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr degrees_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr status_timer_; // 상태 확인 타이머 추가
    std::vector<int> degrees_;
    sensor_msgs::msg::LaserScan latest_scan_;
    std::vector<bool> obstacle_states_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto a = std::make_shared<laser_read>();
    RCLCPP_INFO(a->get_logger(), "구독자 노드 스핀 시작!");
    rclcpp::spin(a);
    rclcpp::shutdown();
    return 0;
}