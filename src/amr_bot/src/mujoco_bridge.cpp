#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

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

        // 500 Hz sim step, matching the plan's SIM_HZ.
        timer_ = create_wall_timer(2ms, std::bind(&MujocoBridge::step_sim, this));

        RCLCPP_INFO(get_logger(), "mujoco_bridge up, stepping at 500 Hz");
    }

    ~MujocoBridge() override {
        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);
    }

private:
    void step_sim() {
        mj_step(m_, d_);
    }

    mjModel* m_ = nullptr;
    mjData* d_ = nullptr;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MujocoBridge>());
    rclcpp::shutdown();
    return 0;
}
