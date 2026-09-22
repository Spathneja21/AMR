#include <mujoco/mujoco.h>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

class MujocoBridge : public rclcpp::Node {
public:
    MujocoBridge() : Node("mujoco_bridge") {
        const char* path = "/ros2_ws/src/amr_bot/models/world/world.xml";

        char error[1000] = "";
        m_ = mj_loadXML(path, nullptr, error, 1000);
        if (!m_) {
            RCLCPP_ERROR(get_logger(), "mj_loadXML failed: %s", error);
            rclcpp::shutdown();
            return;
        }
        d_ = mj_makeData(m_);

        left_joint_id_ = mj_name2id(m_, mjOBJ_JOINT, "left_wheel_joint");
        right_joint_id_ = mj_name2id(m_, mjOBJ_JOINT, "right_wheel_joint");

        int lidar0_id = mj_name2id(m_, mjOBJ_SENSOR, "lidar_0");
        lidar_sensor_adr_ = m_->sensor_adr[lidar0_id];

        // 500 Hz sim step, matching the plan's SIM_HZ.
        timer_ = create_wall_timer(2ms, std::bind(&MujocoBridge::step_sim, this));
        state_timer_ = create_wall_timer(50ms, std::bind(&MujocoBridge::publish_state, this));

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MujocoBridge::on_cmd_vel, this, std::placeholders::_1));

        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(get_logger(), "mujoco_bridge up, stepping at 500 Hz");
    }

    ~MujocoBridge() override {
        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);
    }

private:
    static constexpr double WHEEL_SEPARATION = 0.205;  // m
    static constexpr double WHEEL_RADIUS = 0.035;       // m

    void step_sim() {
        mj_step(m_, d_);
    }

    void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double v = msg->linear.x;
        double w = msg->angular.z;

        // ctrl[0] = left_wheel_vel, ctrl[1] = right_wheel_vel — this order
        // comes from the <actuator> declaration order in world.xml, not the name.
        d_->ctrl[0] = (v - w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS;
        d_->ctrl[1] = (v + w * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS;
    }

    void publish_state() {
        rclcpp::Time now = get_clock()->now();
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = d_->qpos[0];
        t.transform.translation.y = d_->qpos[1];
        t.transform.translation.z = d_->qpos[2];
        t.transform.rotation.w = d_->qpos[3];
        t.transform.rotation.x = d_->qpos[4];
        t.transform.rotation.y = d_->qpos[5];
        t.transform.rotation.z = d_->qpos[6];
        tf_broadcaster_->sendTransform(t);


        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = d_->qpos[0];
        odom.pose.pose.position.y = d_->qpos[1];
        odom.pose.pose.position.z = d_->qpos[2];
        odom.pose.pose.orientation = t.transform.rotation;   // reuse, don't recompute
        odom.twist.twist.linear.x = d_ ->qvel[0];
        odom.twist.twist.angular.z = d_ ->qvel[5];
        odom_pub_->publish(odom);


        sensor_msgs::msg::JointState js;
        js.header.stamp = now;
        js.name = {"left_wheel_joint", "right_wheel_joint"};
        js.position = {
            d_->qpos[m_->jnt_qposadr[left_joint_id_]],
            d_->qpos[m_->jnt_qposadr[right_joint_id_]]
        };
        js.velocity = {
            d_->qvel[m_->jnt_dofadr[left_joint_id_]],
            d_->qvel[m_->jnt_dofadr[right_joint_id_]]
        };

        joint_pub_->publish(js);

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = now;
        scan.header.frame_id = "lidar_link";
        scan.angle_min = -M_PI;
        scan.angle_increment = 2.0 * M_PI / 360.0;
        scan.angle_max = scan.angle_min + 359 * scan.angle_increment;
        scan.range_min = 0.1;
        scan.range_max = 10.0;
        scan.ranges.resize(360);
        for (int i = 0; i < 360; i++) {
            double r = d_->sensordata[lidar_sensor_adr_ + i];
            scan.ranges[i] = (r < 0) ? std::numeric_limits<double>::infinity() : r;
        }       
        scan_pub_->publish(scan);
    }

    mjModel* m_ = nullptr;
    mjData* d_ = nullptr;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    int left_joint_id_, right_joint_id_;
    int lidar_sensor_adr_ ;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MujocoBridge>());
    rclcpp::shutdown();
    return 0;
}
