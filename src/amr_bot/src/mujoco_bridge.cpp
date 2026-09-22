#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <limits>
#include <mutex>
#include <thread>
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

        init_viewer();

        RCLCPP_INFO(get_logger(), "mujoco_bridge up, stepping at 500 Hz");
    }

    ~MujocoBridge() override {
        if (window_) {
            mjv_freeScene(&scn_);
            mjr_freeContext(&con_);
            glfwDestroyWindow(window_);
            glfwTerminate();
        }
        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);
    }

    // Runs the GLFW render loop on the calling thread. Blocks until the
    // window is closed or ROS shuts down — call this from main(), never
    // from inside an executor callback.
    void run_viewer() {
        if (!window_) return;

        while (!glfwWindowShouldClose(window_) && rclcpp::ok()) {
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

            // mjv_updateScene reads d_ (qpos/qvel/etc.) to build the draw
            // list — this is the one place that races against step_sim(),
            // which writes d_ from the ROS executor thread. Lock only
            // around the read; mjr_render/glfwSwapBuffers only touch scn_,
            // which is now a self-contained snapshot.
            {
                std::lock_guard<std::mutex> lock(sim_mutex_);
                mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
            }
            mjr_render(viewport, &scn_, &con_);

            glfwSwapBuffers(window_);
            glfwPollEvents();
        }
    }

private:
    static constexpr double WHEEL_SEPARATION = 0.205;  // m
    static constexpr double WHEEL_RADIUS = 0.035;       // m

    void init_viewer() {
        if (!glfwInit()) {
            RCLCPP_ERROR(get_logger(), "glfwInit failed, running without a viewer window");
            return;
        }
        window_ = glfwCreateWindow(1200, 900, "mujoco_bridge (live)", nullptr, nullptr);
        if (!window_) {
            RCLCPP_ERROR(get_logger(), "glfwCreateWindow failed, running without a viewer window");
            glfwTerminate();
            return;
        }
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);

        // GLFW callbacks are plain C function pointers, not member
        // functions, so `this` is stashed on the window and recovered
        // inside each static callback via glfwGetWindowUserPointer.
        glfwSetWindowUserPointer(window_, this);
        glfwSetMouseButtonCallback(window_, &MujocoBridge::mouse_button_cb);
        glfwSetCursorPosCallback(window_, &MujocoBridge::cursor_pos_cb);
        glfwSetScrollCallback(window_, &MujocoBridge::scroll_cb);

        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);

        mjv_makeScene(m_, &scn_, 2000);
        mjr_makeContext(m_, &con_, mjFONTSCALE_150);

        // Frame the whole loaded model automatically using MuJoCo's own
        // computed bounding stats, instead of guessing a camera position.
        cam_.type = mjCAMERA_FREE;
        cam_.lookat[0] = m_->stat.center[0];
        cam_.lookat[1] = m_->stat.center[1];
        cam_.lookat[2] = m_->stat.center[2];
        cam_.distance = 1.5 * m_->stat.extent;
        cam_.azimuth = 90;
        cam_.elevation = -30;
    }

    // Mouse-orbit camera controls, same scheme as MuJoCo's own sample
    // viewer: left-drag rotates, right-drag pans, scroll zooms, shift
    // switches the drag to its horizontal variant. These callbacks fire
    // from glfwPollEvents() inside run_viewer(), i.e. on the SAME thread
    // that reads cam_ for rendering — no mutex needed here, unlike d_.
    static void mouse_button_cb(GLFWwindow* window, int /*button*/, int /*action*/, int /*mods*/) {
        auto* self = static_cast<MujocoBridge*>(glfwGetWindowUserPointer(window));
        self->button_left_   = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS;
        self->button_middle_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
        self->button_right_  = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS;
        glfwGetCursorPos(window, &self->last_x_, &self->last_y_);
    }

    static void cursor_pos_cb(GLFWwindow* window, double xpos, double ypos) {
        auto* self = static_cast<MujocoBridge*>(glfwGetWindowUserPointer(window));
        if (!self->button_left_ && !self->button_middle_ && !self->button_right_) {
            self->last_x_ = xpos;
            self->last_y_ = ypos;
            return;
        }

        double dx = xpos - self->last_x_;
        double dy = ypos - self->last_y_;
        self->last_x_ = xpos;
        self->last_y_ = ypos;

        int width, height;
        glfwGetWindowSize(window, &width, &height);

        bool mod_shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                          glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

        mjtMouse action;
        if (self->button_right_) {
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;      // pan
        } else if (self->button_left_) {
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;  // orbit
        } else {
            action = mjMOUSE_ZOOM;                                     // middle-drag
        }

        mjv_moveCamera(self->m_, action, dx / height, dy / height, &self->cam_);
    }

    static void scroll_cb(GLFWwindow* window, double /*xoffset*/, double yoffset) {
        auto* self = static_cast<MujocoBridge*>(glfwGetWindowUserPointer(window));
        mjv_moveCamera(self->m_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &self->cam_);
    }

    void step_sim() {
        std::lock_guard<std::mutex> lock(sim_mutex_);
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
        js.name = {"left_wheel_link_to_chassis_link_joint", "right_wheel_link_to_chassis_link_joint"};
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
    int lidar_sensor_adr_;

    // Guards d_ (mjData) across the ROS executor thread (step_sim writes)
    // and the render loop, which runs on a separate thread (main()) and
    // reads d_ via mjv_updateScene. Nothing else needs it — callbacks
    // within the executor are already serialized by the single-threaded
    // spin in main(), so this mutex exists ONLY for the executor-vs-render
    // thread boundary, not between ROS callbacks themselves.
    std::mutex sim_mutex_;

    GLFWwindow* window_ = nullptr;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    bool button_left_ = false, button_middle_ = false, button_right_ = false;
    double last_x_ = 0.0, last_y_ = 0.0;
    mjrContext con_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MujocoBridge>();

    // ROS spins on a background thread; the GLFW/OpenGL render loop stays
    // on main() — GLFW window/context calls should stay on the thread that
    // created the window.
    std::thread spin_thread([&node]() { rclcpp::spin(node); });

    node->run_viewer();

    rclcpp::shutdown();   // makes the blocked spin() in spin_thread return
    spin_thread.join();
    return 0;
}
