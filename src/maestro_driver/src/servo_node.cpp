#include <rclcpp/rclcpp.hpp>
#include <crab_msgs/msg/legs_joints_state.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include "PolstroSerialInterface.h"

const int rotation_direction[18] = {
    1, -1, 1,
    1, -1, 1,
    1, -1, 1,
    1, 1, -1,
    1, 1, -1,
    1, 1, -1
};

class MaestroController : public rclcpp::Node {
public:
    MaestroController() : Node("maestro_servo_node"), initialized_(false), init_step_(0) {
        this->declare_parameter<double>("joint_lower_limit", -1.570796327);
        this->declare_parameter<double>("joint_upper_limit", 1.570796327);
        this->declare_parameter<std::string>("port_name", "/dev/ttyS5");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<double>("max_speed", 5.0);  // max change per update

        joint_lower_limit_ = this->get_parameter("joint_lower_limit").as_double();
        joint_upper_limit_ = this->get_parameter("joint_upper_limit").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        limit_coef_ = 127 / ((joint_upper_limit_ - joint_lower_limit_) / 2);

        // Initialize current positions - assume servos might be anywhere
        // We'll smoothly move them to neutral during initialization
        for (int i = 0; i < 18; i++) {
            current_pos_[i] = -1.0f;  // -1 means unknown position
            target_pos_[i] = 127.0f;  // Target is neutral
        }

        std::string port_name = this->get_parameter("port_name").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        maestro_.reset(Polstro::SerialInterface::createSerialInterface(port_name, baud_rate));

        if (maestro_ && maestro_->isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Maestro servo controller connected on %s", port_name.c_str());
            // Start smooth initialization timer
            init_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(30),
                std::bind(&MaestroController::initTimerCallback, this));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open Maestro on %s", port_name.c_str());
        }

        sub_ = this->create_subscription<crab_msgs::msg::LegsJointsState>(
            "joints_to_controller", 1,
            std::bind(&MaestroController::chatterLegsState, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Maestro servo controller is ready...");
    }

    ~MaestroController() = default;

private:
    void initTimerCallback() {
        if (!maestro_ || !maestro_->isOpen()) return;
        
        if (init_step_ == 0) {
            RCLCPP_INFO(this->get_logger(), "Starting smooth servo initialization...");
            // First step: assume current position is neutral (127)
            // This prevents sudden jumps
            for (int i = 0; i < 18; i++) {
                current_pos_[i] = 127.0f;
            }
            init_step_++;
            return;
        }
        
        // Gradually move all servos to neutral over ~60 steps (1.8 seconds)
        bool all_done = true;
        for (int i = 0; i < 18; i++) {
            float diff = target_pos_[i] - current_pos_[i];
            if (std::abs(diff) > 0.5f) {
                all_done = false;
                // Move slowly toward target
                if (std::abs(diff) > 2.0f) {
                    current_pos_[i] += (diff > 0) ? 2.0f : -2.0f;
                } else {
                    current_pos_[i] = target_pos_[i];
                }
            }
            maestro_->setTargetMSS(i, static_cast<unsigned char>(current_pos_[i]));
        }
        
        if (all_done) {
            initialized_ = true;
            init_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Servos initialized to neutral position");
        }
        
        init_step_++;
    }

    void chatterLegsState(const crab_msgs::msg::LegsJointsState::SharedPtr legs_jnts) {
        if (!maestro_ || !maestro_->isOpen()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Maestro not open");
            return;
        }
        
        // Wait for initialization to complete
        if (!initialized_) {
            return;
        }

        int s_num;

        for (int i = 0; i < num_legs_; i++) {
            for (int j = 0; j < num_joints_; j++) {
                s_num = i * 3 + j;
                float target = 127.0f + rotation_direction[s_num] * legs_jnts->joints_state[i].joint[j] * limit_coef_;
                target = std::clamp(target, 0.0f, 254.0f);
                
                // Smooth movement: limit speed of change
                float diff = target - current_pos_[s_num];
                if (std::abs(diff) > max_speed_) {
                    current_pos_[s_num] += (diff > 0) ? max_speed_ : -max_speed_;
                } else {
                    current_pos_[s_num] = target;
                }
                
                unsigned char target_byte = static_cast<unsigned char>(current_pos_[s_num]);
                maestro_->setTargetMSS(s_num, target_byte);
            }
        }
    }

    std::unique_ptr<Polstro::SerialInterface> maestro_;
    double joint_lower_limit_, joint_upper_limit_, limit_coef_, max_speed_;
    float current_pos_[18];
    float target_pos_[18];
    bool initialized_;
    int init_step_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    static constexpr unsigned int num_joints_ = 3;
    static constexpr unsigned int num_legs_ = 6;
    rclcpp::Subscription<crab_msgs::msg::LegsJointsState>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MaestroController>());
    rclcpp::shutdown();
    return 0;
}
