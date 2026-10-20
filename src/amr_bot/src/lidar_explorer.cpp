// Frontier explorer: picks the nearest reachable frontier (free cell next to unknown) in the SLAM
// map, follows a grid BFS path toward it, and drives with the lidar gap logic as the local layer.
// Before the first map arrives it falls back to the range-weighted widest-gap rule.
// The travelled path is published on /explorer_path and the current waypoint on /explorer_goal.
#include <algorithm>
#include <cmath>
#include <deque>
#include <queue>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LidarExplorer : public rclcpp::Node {
public:
    LidarExplorer() : Node("lidar_explorer") {
        robot_r_    = declare_parameter("robot_radius", 0.15);
        margin_     = declare_parameter("margin", 0.10);       // extra clearance around obstacles
        obs_dist_   = declare_parameter("obstacle_dist", 1.0); // beams closer than this create a bubble
        stop_dist_  = declare_parameter("stop_dist", 0.30);    // rotate in place if the way ahead is this close
        slow_dist_  = declare_parameter("slow_dist", 1.0);     // full speed beyond this clearance
        weight_pow_ = declare_parameter("weight_pow", 2.0);    // p in w = r^p (fallback rule only)
        head_bonus_ = declare_parameter("heading_bonus", 0.3); // fallback rule: prefer gaps straight ahead
        v_max_      = declare_parameter("v_max", 0.2);
        w_max_      = declare_parameter("w_max", 0.8);
        kp_         = declare_parameter("kp_heading", 1.5);
        turn_only_  = declare_parameter("turn_only_angle", 1.0); // rad: beyond this, stop and rotate
        stuck_win_s_  = declare_parameter("stuck_window_s", 6.0);
        stuck_dist_   = declare_parameter("stuck_dist", 0.15);
        escape_s_     = declare_parameter("escape_s", 2.5);
        inflate_      = declare_parameter("plan_inflation", 0.20); // keep BFS paths this far from walls
        lookahead_    = declare_parameter("waypoint_lookahead", 1.0);
        min_front_d_  = declare_parameter("min_frontier_dist", 0.5);
        min_cluster_  = declare_parameter("min_frontier_cells", 8);
        goal_timeout_ = declare_parameter("goal_timeout_s", 40.0);
        reach_dist_   = declare_parameter("frontier_reached_dist", 0.4);

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/explorer_path", 1);
        goal_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/explorer_goal", 1);
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&LidarExplorer::on_scan, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&LidarExplorer::on_odom, this, std::placeholders::_1));
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).reliable(),
            [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { map_ = m; });
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        path_.header.frame_id = "odom";
        path_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {
            path_.header.stamp = now();
            path_pub_->publish(path_);
        });
        plan_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&LidarExplorer::plan, this));
        RCLCPP_INFO(get_logger(), "lidar_explorer up (frontier mode): v_max %.2f w_max %.2f", v_max_, w_max_);
    }

private:
    struct Pose { double t, x, y; };

    // ---------------- global layer: frontier selection on the SLAM map ----------------
    void plan() {
        if (!map_ || done_) return;
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no map->base_link yet: %s", e.what());
            return;
        }
        const double mx = tf.transform.translation.x, my = tf.transform.translation.y;
        const auto& m = *map_;
        const int W = m.info.width, H = m.info.height;
        const double res = m.info.resolution, ox = m.info.origin.position.x, oy = m.info.origin.position.y;
        const int rx = static_cast<int>(std::floor((mx - ox) / res)), ry = static_cast<int>(std::floor((my - oy) / res));
        if (rx < 0 || ry < 0 || rx >= W || ry >= H) return;
        const auto& g = m.data;
        auto wx = [&](int c) { return ox + (c % W + 0.5) * res; };
        auto wy = [&](int c) { return oy + (c / W + 0.5) * res; };

        // inflate occupied cells so BFS paths keep clear of walls
        std::vector<uint8_t> near_wall(W * H, 0);
        const int k = static_cast<int>(std::ceil(inflate_ / res));
        for (int c = 0; c < W * H; ++c) {
            if (g[c] < 50) continue;
            const int x = c % W, y = c / W;
            for (int yy = std::max(0, y - k); yy <= std::min(H - 1, y + k); ++yy)
                for (int xx = std::max(0, x - k); xx <= std::min(W - 1, x + k); ++xx) near_wall[yy * W + xx] = 1;
        }

        auto is_frontier = [&](int c) {
            if (g[c] != 0) return false;
            const int x = c % W, y = c / W;
            return (x > 0 && g[c - 1] == -1) || (x < W - 1 && g[c + 1] == -1) ||
                   (y > 0 && g[c - W] == -1) || (y < H - 1 && g[c + W] == -1);
        };
        // size of the 8-connected frontier cluster containing c, counted up to `need`
        auto cluster_ok = [&](int c0, int need) {
            std::vector<int> stack{c0};
            std::vector<uint8_t> seen(W * H, 0);
            seen[c0] = 1;
            int count = 0;
            while (!stack.empty() && count < need) {
                const int c = stack.back();
                stack.pop_back();
                ++count;
                const int x = c % W, y = c / W;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int nx = x + dx, ny = y + dy;
                        if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                        const int nc = ny * W + nx;
                        if (!seen[nc] && is_frontier(nc)) { seen[nc] = 1; stack.push_back(nc); }
                    }
            }
            return count >= need;
        };
        auto blacklisted = [&](int c) {
            for (const auto& b : blacklist_)
                if (std::hypot(wx(c) - b.first, wy(c) - b.second) < 0.5) return true;
            return false;
        };

        // BFS through free, non-inflated cells from the robot: the first good frontier is the nearest reachable one
        std::vector<int> parent(W * H, -2);
        std::queue<int> q;
        const int start = ry * W + rx;
        parent[start] = -1;
        q.push(start);
        int found = -1;
        while (!q.empty()) {
            const int c = q.front();
            q.pop();
            if (std::hypot(wx(c) - mx, wy(c) - my) > min_front_d_ && is_frontier(c) && !blacklisted(c) &&
                cluster_ok(c, min_cluster_)) {
                found = c;
                break;
            }
            const int x = c % W, y = c / W;
            const int nb[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
            for (const auto& d : nb) {
                const int nx = x + d[0], ny = y + d[1];
                if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                const int nc = ny * W + nx;
                // near the robot, allow inflated cells so it can leave a wall it is standing next to
                const bool near_start = std::abs(nx - rx) + std::abs(ny - ry) <= 2 * k;
                if (parent[nc] != -2 || g[nc] != 0 || (near_wall[nc] && !near_start)) continue;
                parent[nc] = c;
                q.push(nc);
            }
        }

        int known = 0;
        for (auto v : g) known += v != -1;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "mapped area: %.1f m^2", known * res * res);

        if (found < 0) {
            has_goal_ = false;
            if (++empty_plans_ >= 5) {
                done_ = true;
                RCLCPP_INFO(get_logger(), "exploration complete: no reachable frontiers left (%.1f m^2 mapped)",
                            known * res * res);
            }
            return;
        }
        empty_plans_ = 0;

        // timeout on one frontier -> blacklist it so we do not keep trying the same unreachable spot
        const double fx = wx(found), fy = wy(found), t = now().seconds();
        if (!has_frontier_ || std::hypot(fx - frontier_x_, fy - frontier_y_) > 1.0) {
            frontier_x_ = fx;
            frontier_y_ = fy;
            frontier_t0_ = t;
            has_frontier_ = true;
        } else if (t - frontier_t0_ > goal_timeout_) {
            RCLCPP_WARN(get_logger(), "frontier (%.2f, %.2f) not reached in %.0f s, blacklisting", fx, fy, goal_timeout_);
            blacklist_.emplace_back(fx, fy);
            has_frontier_ = false;
            has_goal_ = false;
            return;
        }

        // waypoint = first cell on the BFS path at least `lookahead` from the robot (or the frontier itself)
        std::vector<int> path;
        for (int c = found; c >= 0; c = parent[c]) path.push_back(c);
        std::reverse(path.begin(), path.end());
        int wp = found;
        for (int c : path)
            if (std::hypot(wx(c) - mx, wy(c) - my) >= lookahead_) { wp = c; break; }
        goal_x_ = wx(wp);
        goal_y_ = wy(wp);
        has_goal_ = std::hypot(fx - mx, fy - my) > reach_dist_;

        geometry_msgs::msg::PointStamped p;
        p.header.frame_id = "map";
        p.header.stamp = now();
        p.point.x = goal_x_;
        p.point.y = goal_y_;
        goal_pub_->publish(p);
    }

    // bearing of the waypoint in the robot frame (needs map->base_link); false if unavailable
    bool goal_bearing(double& bearing) {
        if (!has_goal_) return false;
        try {
            const auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            const auto& q = tf.transform.rotation;
            const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
            bearing = std::remainder(std::atan2(goal_y_ - tf.transform.translation.y,
                                                goal_x_ - tf.transform.translation.x) - yaw, 2 * M_PI);
            return true;
        } catch (const tf2::TransformException&) {
            return false;
        }
    }

    // ---------------- local layer: lidar gaps ----------------
    void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const double t = now().seconds();
        const auto& p = msg->pose.pose;
        hist_.push_back({t, p.position.x, p.position.y});
        while (!hist_.empty() && t - hist_.front().t > stuck_win_s_) hist_.pop_front();

        if (path_.poses.empty() || std::hypot(p.position.x - last_path_x_, p.position.y - last_path_y_) > 0.05) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_.header;
            ps.pose = p;
            path_.poses.push_back(ps);
            last_path_x_ = p.position.x;
            last_path_y_ = p.position.y;
        }
    }

    bool is_stuck(double t) const {
        if (hist_.size() < 2 || hist_.back().t - hist_.front().t < stuck_win_s_ * 0.9) return false;
        if (t < escape_until_ + stuck_win_s_) return false;  // give an escape time to take effect
        return std::hypot(hist_.back().x - hist_.front().x, hist_.back().y - hist_.front().y) < stuck_dist_;
    }

    void on_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        const int n = static_cast<int>(msg->ranges.size());
        if (n < 10) return;
        const double t = now().seconds();
        geometry_msgs::msg::Twist cmd;
        if (done_) {
            cmd_pub_->publish(cmd);
            return;
        }

        // 1. clean: no return -> range_max, too close -> range_min
        std::vector<double> r(n), ang(n);
        for (int i = 0; i < n; ++i) {
            double v = msg->ranges[i];
            if (!std::isfinite(v) || v >= msg->range_max) v = msg->range_max;
            r[i] = std::max(v, static_cast<double>(msg->range_min));
            ang[i] = msg->angle_min + i * msg->angle_increment;
        }

        // 2. bubbles: every close beam blocks the beams within the robot's angular footprint
        std::vector<bool> blocked(n, false);
        const double inc = msg->angle_increment;
        for (int i = 0; i < n; ++i) {
            if (r[i] >= obs_dist_) continue;
            const double half = std::asin(std::min(1.0, (robot_r_ + margin_) / r[i]));
            const int k = static_cast<int>(std::ceil(half / inc));
            for (int j = std::max(0, i - k); j <= std::min(n - 1, i + k); ++j) blocked[j] = true;
        }

        // escape manoeuvre in progress
        if (t < escape_until_) {
            cmd.angular.z = escape_dir_ * w_max_;
            cmd_pub_->publish(cmd);
            return;
        }
        if (is_stuck(t)) {
            double left = 0, right = 0;
            for (int i = 0; i < n; ++i) (ang[i] > 0 ? left : right) += r[i];
            escape_dir_ = left >= right ? 1.0 : -1.0;
            escape_until_ = t + escape_s_;
            RCLCPP_WARN(get_logger(), "stuck: rotating %s for %.1f s", escape_dir_ > 0 ? "left" : "right", escape_s_);
            return;
        }

        // 3. gaps = runs of unblocked beams (>= 3 beams).
        //    With a waypoint: the direction inside any gap closest to the waypoint bearing.
        //    Without one: range-weighted mean angle inside the best (mean range * width) gap.
        double bearing = 0;
        const bool goal = goal_bearing(bearing);
        double best = goal ? 1e9 : -1, target = 0;
        bool any_gap = false;
        for (int i = 0; i < n;) {
            if (blocked[i]) { ++i; continue; }
            int j = i;
            while (j + 1 < n && !blocked[j + 1]) ++j;
            if (j - i + 1 >= 3) {
                any_gap = true;
                if (goal) {
                    const double cand = std::clamp(bearing, ang[i], ang[j]);
                    if (std::fabs(cand - bearing) < best) { best = std::fabs(cand - bearing); target = cand; }
                } else {
                    double sum = 0, sw = 0, swa = 0;
                    for (int k = i; k <= j; ++k) {
                        sum += r[k];
                        const double w = std::pow(r[k], weight_pow_);
                        sw += w;
                        swa += w * ang[k];
                    }
                    const double gap_target = swa / sw;
                    const double score = sum / (j - i + 1) * (j - i + 1) * inc * (1.0 + head_bonus_ * std::cos(gap_target));
                    if (score > best) { best = score; target = gap_target; }
                }
            }
            i = j + 1;
        }

        // nowhere to go: spin toward the more open side
        if (!any_gap) {
            double left = 0, right = 0;
            for (int i = 0; i < n; ++i) (ang[i] > 0 ? left : right) += r[i];
            cmd.angular.z = (left >= right ? 1.0 : -1.0) * w_max_;
            cmd_pub_->publish(cmd);
            return;
        }

        // 4. speed: slow with heading error and with the nearest return around the target direction
        double front = msg->range_max;
        for (int i = 0; i < n; ++i)
            if (std::fabs(ang[i] - target) < 0.45) front = std::min(front, r[i]);

        cmd.angular.z = std::clamp(kp_ * target, -w_max_, w_max_);
        if (std::fabs(target) < turn_only_ && front > stop_dist_) {
            const double clear = std::clamp((front - stop_dist_) / (slow_dist_ - stop_dist_), 0.0, 1.0);
            cmd.linear.x = v_max_ * clear * std::max(0.0, std::cos(target));
        }
        cmd_pub_->publish(cmd);
    }

    double robot_r_, margin_, obs_dist_, stop_dist_, slow_dist_, weight_pow_, head_bonus_;
    double v_max_, w_max_, kp_, turn_only_, stuck_win_s_, stuck_dist_, escape_s_;
    double inflate_, lookahead_, min_front_d_, goal_timeout_, reach_dist_;
    int min_cluster_;
    double escape_until_ = 0, escape_dir_ = 1;
    double last_path_x_ = 0, last_path_y_ = 0;
    double goal_x_ = 0, goal_y_ = 0, frontier_x_ = 0, frontier_y_ = 0, frontier_t0_ = 0;
    bool has_goal_ = false, has_frontier_ = false, done_ = false;
    int empty_plans_ = 0;
    std::vector<std::pair<double, double>> blacklist_;
    std::deque<Pose> hist_;
    nav_msgs::msg::Path path_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr path_timer_, plan_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarExplorer>());
    rclcpp::shutdown();
    return 0;
}
