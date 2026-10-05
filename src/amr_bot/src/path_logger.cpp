#include <rclcpp/rclcpp.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <set>

namespace {
double yaw_of(const geometry_msgs::msg::Quaternion & q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

const char * status_name(int8_t s) {
    switch (s) {
        case 4: return "SUCCEEDED";
        case 5: return "CANCELED";
        case 6: return "ABORTED";
        default: return "UNKNOWN";
    }
}
}  // namespace


// Records one CSV set per navigation goal: the plans the controller was handed
// and the robot's actual motion, so the two can be compared offline.
//   run start : /goal_pose            (RViz "2D Goal Pose" or scripts/send_goal.sh)
//   run end   : terminal status on /navigate_to_pose/_action/status, + tail_s
class PathLogger : public rclcpp::Node {
public:
    PathLogger() : Node("path_logger") {
        label_ = declare_parameter<std::string>("label", "run");
        dir_ = declare_parameter<std::string>("log_dir", "/ros2_ws/runs");
        tail_s_ = declare_parameter<double>("tail_s", 1.0);
        const double rate_hz = declare_parameter<double>("rate_hz", 20.0);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) { on_goal(*m); });
        status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
            "/navigate_to_pose/_action/status", 10,
            [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr m) { on_status(m); });
        plan_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/received_global_plan", 10,
            [this](nav_msgs::msg::Path::ConstSharedPtr m) { on_plan(*m); });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { odom_ = *m; });
        cmd_nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10,
            [this](geometry_msgs::msg::Twist::ConstSharedPtr m) { cmd_nav_ = *m; });
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](geometry_msgs::msg::Twist::ConstSharedPtr m) { cmd_ = *m; });

        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz),
                                   [this]() { on_tick(); });

        RCLCPP_INFO(get_logger(), "path_logger ready: label=%s dir=%s, waiting for /goal_pose",
                    label_.c_str(), dir_.c_str());
    }

    ~PathLogger() override {
        if (active_) finish_run("INTERRUPTED");
    }

    private:
    void on_goal(const geometry_msgs::msg::PoseStamped & g) {
        if (active_) finish_run("PREEMPTED");
        start_run(g);
    }

    void start_run(const geometry_msgs::msg::PoseStamped & g) {
        std::filesystem::create_directories(dir_);

        // First free index for this label, so restarting the node never overwrites a run.
        char name[96];
        for (int n = 1;; ++n) {
            std::snprintf(name, sizeof name, "%s_%03d", label_.c_str(), n);
            if (!std::filesystem::exists(dir_ + "/" + name + "_traj.csv")) break;
        }
        prefix_ = dir_ + "/" + name;

        traj_.open(prefix_ + "_traj.csv");
        plans_.open(prefix_ + "_plans.csv");
        if (!traj_ || !plans_) {
            RCLCPP_ERROR(get_logger(), "cannot write under %s", dir_.c_str());
            return;
        }
        traj_ << std::fixed << std::setprecision(5);
        plans_ << std::fixed << std::setprecision(5);
        traj_ << "t,x_map,y_map,yaw_map,x_gt,y_gt,yaw_gt,v_odom,w_odom,"
                 "v_cmd_nav,w_cmd_nav,v_cmd,w_cmd\n";
        plans_ << "plan_id,t_recv,i,x,y,yaw\n";

        goal_ = g;
        t0_ = now();
        active_ = true;
        ending_ = false;
        plan_id_ = -1;
        nav_id_.reset();
        known_ids_.clear();
        if (last_status_) {
            for (const auto & s : last_status_->status_list) {
                known_ids_.insert(s.goal_info.goal_id.uuid);   // goals from earlier runs
            }
        }
        if (g.header.frame_id != "map") {
            RCLCPP_WARN(get_logger(), "goal frame is '%s', not 'map' - set RViz Fixed Frame to map",
                        g.header.frame_id.c_str());
        }
        RCLCPP_INFO(get_logger(), "run started: %s (goal x=%.2f y=%.2f yaw=%.2f)", name,
                    g.pose.position.x, g.pose.position.y, yaw_of(g.pose.orientation));
    }

    void on_status(action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
        last_status_ = msg;
        if (!active_ || ending_) return;
        for (const auto & s : msg->status_list) {
            const auto & id = s.goal_info.goal_id.uuid;
            if (known_ids_.count(id)) continue;
            if (!nav_id_) nav_id_ = id;    // the NavigateToPose goal created for this run
            if (*nav_id_ == id && s.status >= action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
                ending_ = true;
                pending_status_ = status_name(s.status);
                end_deadline_ = now() + rclcpp::Duration::from_seconds(tail_s_);
                break;
            }
        }
    }

    void on_plan(const nav_msgs::msg::Path & p) {
        if (!active_ || ending_) return;
        ++plan_id_;
        const double t = (now() - t0_).seconds();
        for (size_t i = 0; i < p.poses.size(); ++i) {
            const auto & q = p.poses[i].pose;
            plans_ << plan_id_ << ',' << t << ',' << i << ',' << q.position.x << ','
                   << q.position.y << ',' << yaw_of(q.orientation) << '\n';
        }
        plans_.flush();
    }

    void on_tick() {
        if (!active_) return;

        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "no map->base_link yet (%s)", e.what());
            return;
        }

        const double nan = std::numeric_limits<double>::quiet_NaN();
        double xg = nan, yg = nan, yawg = nan, vo = nan, wo = nan;
        if (odom_) {
            xg = odom_->pose.pose.position.x;
            yg = odom_->pose.pose.position.y;
            yawg = yaw_of(odom_->pose.pose.orientation);
            vo = odom_->twist.twist.linear.x;
            wo = odom_->twist.twist.angular.z;
        }

        traj_ << (now() - t0_).seconds() << ','
              << tf.transform.translation.x << ',' << tf.transform.translation.y << ','
              << yaw_of(tf.transform.rotation) << ','
              << xg << ',' << yg << ',' << yawg << ',' << vo << ',' << wo << ','
              << cmd_nav_.linear.x << ',' << cmd_nav_.angular.z << ','
              << cmd_.linear.x << ',' << cmd_.angular.z << '\n';
        traj_.flush();

        if (ending_ && now() >= end_deadline_) finish_run(pending_status_);
    }

    void finish_run(const std::string & status) {
        traj_.close();
        plans_.close();
        std::ofstream g(prefix_ + "_goal.csv");
        g << std::fixed << std::setprecision(5)
          << "goal_x,goal_y,goal_yaw,goal_frame,t_start_abs,t_end,end_status,n_plans\n"
          << goal_.pose.position.x << ',' << goal_.pose.position.y << ','
          << yaw_of(goal_.pose.orientation) << ',' << goal_.header.frame_id << ','
          << t0_.seconds() << ',' << (now() - t0_).seconds() << ',' << status << ','
          << plan_id_ + 1 << '\n';
        active_ = false;
        ending_ = false;
        RCLCPP_INFO(get_logger(), "run finished: %s (%s, %d plans)", prefix_.c_str(),
                    status.c_str(), plan_id_ + 1);
    }

    std::string label_, dir_, prefix_, pending_status_;
    double tail_s_ = 1.0;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_nav_sub_, cmd_sub_;

    std::optional<nav_msgs::msg::Odometry> odom_;
    geometry_msgs::msg::Twist cmd_nav_, cmd_;                // zero until the first message
    action_msgs::msg::GoalStatusArray::ConstSharedPtr last_status_;

    bool active_ = false, ending_ = false;
    int plan_id_ = -1;
    rclcpp::Time t0_, end_deadline_;
    geometry_msgs::msg::PoseStamped goal_;
    std::optional<std::array<uint8_t, 16>> nav_id_;
    std::set<std::array<uint8_t, 16>> known_ids_;
    std::ofstream traj_, plans_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathLogger>());
    rclcpp::shutdown();
    return 0;
}