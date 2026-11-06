// Camera for the Gazebo backend: Gazebo's own camera sensor crashes the server here (PROCESS.md), so
// this renders the robot's onboard_cam from the MuJoCo office scene at the true pose reported by
// Gazebo, and publishes /camera/image_raw. No MuJoCo physics runs here; the model is only posed and drawn.
// ponytail: shows the MuJoCo office, so objects added to the Gazebo world at runtime don't appear.
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class GazeboCamera : public rclcpp::Node {
public:
    GazeboCamera() : Node("gazebo_camera") {
        const std::string scene = declare_parameter<std::string>(
            "scene", "/ros2_ws/src/amr_bot/models/world/office_scene.xml");
        period_ = 1.0 / declare_parameter("camera_hz", 10.0);
        width_ = declare_parameter("width", 640);
        height_ = declare_parameter("height", 480);

        char error[1000] = "";
        m_ = mj_loadXML(scene.c_str(), nullptr, error, 1000);
        if (!m_) {
            RCLCPP_ERROR(get_logger(), "mj_loadXML failed: %s", error);
            return;
        }
        d_ = mj_makeData(m_);
        const int cam_id = mj_name2id(m_, mjOBJ_CAMERA, "onboard_cam");
        const int body_id = mj_name2id(m_, mjOBJ_BODY, "base_link");
        if (cam_id < 0 || body_id < 0 || m_->body_jntnum[body_id] < 1) {
            RCLCPP_ERROR(get_logger(), "onboard_cam or the base_link free joint is missing from %s", scene.c_str());
            return;
        }
        qadr_ = m_->jnt_qposadr[m_->body_jntadr[body_id]];

        // An offscreen buffer still needs a GL context, and GLFW only gives one with a window: keep it hidden.
        if (!glfwInit()) {
            RCLCPP_ERROR(get_logger(), "glfwInit failed (is DISPLAY set?)");
            return;
        }
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window_ = glfwCreateWindow(64, 64, "gazebo_camera", nullptr, nullptr);
        if (!window_) {
            RCLCPP_ERROR(get_logger(), "glfwCreateWindow failed");
            return;
        }
        glfwMakeContextCurrent(window_);

        m_->vis.global.offwidth = width_;  // must be set before mjr_makeContext allocates the buffer
        m_->vis.global.offheight = height_;
        // znear is a fraction of the model's size: for a whole office it clips the floor just ahead of
        // the robot. Use 0.05 m, the near clip of the camera in the URDF.
        m_->vis.map.znear = 0.05 / m_->stat.extent;
        mjv_defaultOption(&opt_);
        opt_.flags[mjVIS_RANGEFINDER] = 0;  // no lidar lines in the camera image
        mjv_defaultCamera(&cam_);
        cam_.type = mjCAMERA_FIXED;
        cam_.fixedcamid = cam_id;
        mjv_defaultScene(&scn_);
        mjv_makeScene(m_, &scn_, 2000);
        mjr_defaultContext(&con_);
        mjr_makeContext(m_, &con_, mjFONTSCALE_100);
        mjr_setBuffer(mjFB_OFFSCREEN, &con_);
        rgb_.resize(static_cast<size_t>(width_) * height_ * 3);

        image_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 5);
        pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth/odom", 10, std::bind(&GazeboCamera::on_pose, this, std::placeholders::_1));
        ready_ = true;
        RCLCPP_INFO(get_logger(), "gazebo_camera up: %dx%d at %.0f Hz", width_, height_, 1.0 / period_);
    }

    ~GazeboCamera() override {
        if (window_) {
            if (ready_) {
                mjv_freeScene(&scn_);
                mjr_freeContext(&con_);
            }
            glfwDestroyWindow(window_);
            glfwTerminate();
        }
        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);
    }

    bool ready() const { return ready_; }

private:
    // Runs on the main thread (rclcpp::spin in main), the thread that owns the GL context.
    void on_pose(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const double t = rclcpp::Time(msg->header.stamp).seconds();
        if (t < last_t_) last_t_ = -1.0;  // world restarted
        if (last_t_ >= 0 && t - last_t_ < period_ - 1e-3) return;
        last_t_ = t;

        const auto& p = msg->pose.pose;
        mjtNum* q = d_->qpos + qadr_;
        q[0] = p.position.x;
        q[1] = p.position.y;
        q[2] = p.position.z;
        q[3] = p.orientation.w;
        q[4] = p.orientation.x;
        q[5] = p.orientation.y;
        q[6] = p.orientation.z;
        // Only what drawing needs: mj_forward would also run contacts and all 360 rangefinders.
        mj_kinematics(m_, d_);
        mj_comPos(m_, d_);
        mj_camlight(m_, d_);

        mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
        const mjrRect viewport = {0, 0, width_, height_};
        mjr_render(viewport, &scn_, &con_);
        mjr_readPixels(rgb_.data(), nullptr, viewport, &con_);

        sensor_msgs::msg::Image img;
        img.header.stamp = msg->header.stamp;
        img.header.frame_id = "camera_link";
        img.height = height_;
        img.width = width_;
        img.encoding = "rgb8";
        img.step = width_ * 3;
        img.data.resize(rgb_.size());
        // mjr_readPixels returns row 0 = bottom; ROS images have row 0 = top.
        for (int row = 0; row < height_; ++row) {
            std::copy_n(rgb_.data() + static_cast<size_t>(height_ - 1 - row) * img.step, img.step,
                        img.data.begin() + static_cast<size_t>(row) * img.step);
        }
        image_pub_->publish(img);
    }

    mjModel* m_ = nullptr;
    mjData* d_ = nullptr;
    GLFWwindow* window_ = nullptr;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;
    int qadr_ = 0, width_ = 640, height_ = 480;
    double period_ = 0.1, last_t_ = -1.0;
    bool ready_ = false;
    std::vector<unsigned char> rgb_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int rc = 1;
    {
        auto node = std::make_shared<GazeboCamera>();
        if (node->ready()) {
            rclcpp::spin(node);
            rc = 0;
        }
    }
    rclcpp::shutdown();
    return rc;
}
