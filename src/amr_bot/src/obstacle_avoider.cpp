#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>

class ObstacleAvoider : public rclcpp::Node {
public:
    ObstacleAvoider() : Node("obstacle_avoider") {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ObstacleAvoider::on_scan, this, std::placeholders::_1));
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe", 10);
    }

private:
    static constexpr double STOP_DISTANCE = 0.3;  // m
    // TODO: two constants, the ray index range for your ±20° forward cone
    static constexpr int FORWARD_LO = 160;
    static constexpr int FORWARD_HI = 200;
    
    void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        double min_range = std::numeric_limits<double>::infinity();
        for (int i = FORWARD_LO; i <= FORWARD_HI; i++) {
            double r = msg->ranges[i];
            if (std::isfinite(r) && r < min_range) {
                min_range = r;
            }
        }

        if (min_range < STOP_DISTANCE) {
            geometry_msgs::msg::Twist stop;  // zero-initialized = all zeros
            cmd_pub_->publish(stop);
            RCLCPP_WARN(get_logger(), "obstacle at %.2fm, stopping", min_range);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoider>());
    rclcpp::shutdown();
    return 0;
}
