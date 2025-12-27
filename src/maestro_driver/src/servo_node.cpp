#include <rclcpp/rclcpp.hpp>
#include <crab_msgs/msg/legs_joints_state.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <thread>
#include "PolstroSerialInterface.h"

const int rotation_direction[18] = {
    1, -1, 1,
    1, -1, 1,
    1, -1, 1,
    1, 1, -1,
    1, 1, -1,
    1, 1, -1
};

// Задержка между командами для предотвращения переполнения буфера Maestro (микросекунды)
constexpr int INTER_COMMAND_DELAY_US = 200;

class MaestroController : public rclcpp::Node {
public:
    MaestroController() : Node("maestro_servo_node"), initialized_(false), init_step_(0), current_init_servo_(0) {
        this->declare_parameter<double>("joint_lower_limit", -1.570796327);
        this->declare_parameter<double>("joint_upper_limit", 1.570796327);
        this->declare_parameter<std::string>("port_name", "/dev/ttyS5");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<double>("max_speed", 5.0);  // max change per update
        // Maestro serial protocol:
        // - "mini_ssc": MiniSSC protocol (0xFF ...) expects normalized 0..254. (default for backwards-compat)
        // - "compact": Pololu Maestro Compact Protocol (0x84 ...) expects target in quarter-microseconds.
        this->declare_parameter<std::string>("protocol", "mini_ssc");
        // Only used for protocol=compact (quarter-microseconds, e.g. 4000..8000 with center 6000)
        this->declare_parameter<int>("target_min", 4000);
        this->declare_parameter<int>("target_max", 8000);
        this->declare_parameter<int>("target_center", 6000);

        joint_lower_limit_ = this->get_parameter("joint_lower_limit").as_double();
        joint_upper_limit_ = this->get_parameter("joint_upper_limit").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        protocol_ = this->get_parameter("protocol").as_string();
        target_min_ = this->get_parameter("target_min").as_int();
        target_max_ = this->get_parameter("target_max").as_int();
        target_center_ = this->get_parameter("target_center").as_int();

        // Scale from joint angle to normalized [-127..127] for MiniSSC (0..254)
        limit_coef_ = 127 / ((joint_upper_limit_ - joint_lower_limit_) / 2);

        // Initialize current positions - assume servos might be anywhere
        // We'll smoothly move them to neutral during initialization
        for (int i = 0; i < 18; i++) {
            current_pos_[i] = -1.0f;  // -1 means unknown position
            target_pos_[i] = 127.0f;  // Target is neutral for MiniSSC mode
        }

        std::string port_name = this->get_parameter("port_name").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        maestro_ = Polstro::SerialInterface::createSerialInterfaceUnique(port_name, baud_rate);

        if (maestro_ && maestro_->isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Maestro servo controller connected on %s", port_name.c_str());
            RCLCPP_INFO(this->get_logger(), "Maestro protocol=%s (compact uses %d..%d center=%d)",
                        protocol_.c_str(), target_min_, target_max_, target_center_);
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
    void sendTarget(int channel, float normalized_or_target, bool add_delay = true) {
        if (!maestro_ || !maestro_->isOpen()) return;

        if (protocol_ == "mini_ssc") {
            unsigned char v = static_cast<unsigned char>(std::clamp(normalized_or_target, 0.0f, 254.0f));
            maestro_->setTargetMSS(static_cast<unsigned char>(channel), v);
        } else {
            // Default: compact protocol target in quarter-microseconds
            unsigned short t = static_cast<unsigned short>(
                std::clamp(normalized_or_target, static_cast<float>(target_min_), static_cast<float>(target_max_)));
            maestro_->setTarget(static_cast<unsigned char>(channel), t);
        }
        
        // Небольшая задержка между командами для предотвращения переполнения буфера Maestro
        if (add_delay) {
            std::this_thread::sleep_for(std::chrono::microseconds(INTER_COMMAND_DELAY_US));
        }
    }

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
            current_init_servo_ = 0;
            return;
        }
        
        // Инициализируем по одному серво за раз для снижения нагрузки на шину и питание
        if (current_init_servo_ < 18) {
            int i = current_init_servo_;
            float diff = target_pos_[i] - current_pos_[i];
            if (std::abs(diff) > 0.5f) {
                // Move slowly toward target
                if (std::abs(diff) > 1.0f) {
                    current_pos_[i] += (diff > 0) ? 1.0f : -1.0f;
                } else {
                    current_pos_[i] = target_pos_[i];
                }
            }
            
            if (protocol_ == "mini_ssc") {
                sendTarget(i, current_pos_[i], false);
            } else {
                sendTarget(i, static_cast<float>(target_center_), false);
            }
            
            // Переходим к следующему серво только когда текущий достиг цели
            if (std::abs(target_pos_[i] - current_pos_[i]) <= 0.5f) {
                current_init_servo_++;
                if (current_init_servo_ < 18) {
                    RCLCPP_DEBUG(this->get_logger(), "Initializing servo %d/%d", current_init_servo_ + 1, 18);
                }
            }
            return;
        }
        
        // Все сервы инициализированы
        initialized_ = true;
        init_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "All servos initialized to neutral position");
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
                float target;
                if (protocol_ == "mini_ssc") {
                    target = 127.0f + rotation_direction[s_num] * legs_jnts->joints_state[i].joint[j] * limit_coef_;
                    target = std::clamp(target, 0.0f, 254.0f);
                } else {
                    // compact: map joint angle to pulse width around center
                    const double half_range = (joint_upper_limit_ - joint_lower_limit_) / 2.0;
                    const double amplitude = (target_max_ - target_min_) / 2.0;
                    const double norm = std::clamp(legs_jnts->joints_state[i].joint[j] / half_range, -1.0, 1.0);
                    target = static_cast<float>(target_center_ + rotation_direction[s_num] * norm * amplitude);
                    target = std::clamp(target, static_cast<float>(target_min_), static_cast<float>(target_max_));
                }
                
                // Smooth movement: limit speed of change
                float diff = target - current_pos_[s_num];
                if (std::abs(diff) > max_speed_) {
                    current_pos_[s_num] += (diff > 0) ? max_speed_ : -max_speed_;
                } else {
                    current_pos_[s_num] = target;
                }
                
                sendTarget(s_num, current_pos_[s_num]);
            }
        }

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "ch0=%.1f ch1=%.1f ch2=%.1f protocol=%s",
                              current_pos_[0], current_pos_[1], current_pos_[2], protocol_.c_str());
    }

    std::unique_ptr<Polstro::SerialInterface> maestro_;
    double joint_lower_limit_, joint_upper_limit_, limit_coef_, max_speed_;
    float current_pos_[18];
    float target_pos_[18];
    std::string protocol_;
    int target_min_, target_max_, target_center_;
    bool initialized_;
    int init_step_;
    int current_init_servo_;
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
