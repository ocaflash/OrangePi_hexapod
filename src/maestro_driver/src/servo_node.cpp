#include <rclcpp/rclcpp.hpp>
#include <crab_msgs/msg/legs_joints_state.hpp>
#include <algorithm>
#include <thread>
#include <chrono>
#include "PolstroSerialInterface.h"

const int rotation_direction[18] = {
    // R1: coxa, femur, tibia (каналы 0, 1, 2)
    1, -1, 1,
    // R2: coxa, femur, tibia (каналы 3, 4, 5)
    1, -1, 1,
    // R3: coxa, femur, tibia (каналы 6, 7, 8)
    1, -1, 1,
    // L1: coxa, femur, tibia (каналы 9, 10, 11)
    -1, 1, -1,
    // L2: coxa, femur, tibia (каналы 12, 13, 14)
    -1, 1, -1,
    // L3: coxa, femur, tibia (каналы 15, 16, 17)
    -1, 1, -1
};

class MaestroController : public rclcpp::Node {
public:
    MaestroController() : Node("maestro_servo_node") {
        this->declare_parameter<double>("joint_lower_limit", -1.570796327);
        this->declare_parameter<double>("joint_upper_limit", 1.570796327);
        this->declare_parameter<std::string>("port_name", "/dev/ttyS5");
        this->declare_parameter<int>("baud_rate", 115200);

        joint_lower_limit_ = this->get_parameter("joint_lower_limit").as_double();
        joint_upper_limit_ = this->get_parameter("joint_upper_limit").as_double();
        limit_coef_ = 127 / ((joint_upper_limit_ - joint_lower_limit_) / 2);

        std::string port_name = this->get_parameter("port_name").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        maestro_ = Polstro::SerialInterface::createSerialInterface(port_name, baud_rate);

        if (maestro_ && maestro_->isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Maestro servo controller connected on %s @ %d baud", 
                        port_name.c_str(), baud_rate);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open Maestro on %s", port_name.c_str());
        }

        sub_ = this->create_subscription<crab_msgs::msg::LegsJointsState>(
            "joints_to_controller", 1,
            std::bind(&MaestroController::chatterLegsState, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Maestro servo controller is ready...");
    }

    ~MaestroController() {
        if (maestro_) {
            delete maestro_;
        }
    }

private:
    void chatterLegsState(const crab_msgs::msg::LegsJointsState::SharedPtr legs_jnts) {
        if (!maestro_ || !maestro_->isOpen()) return;

        float target_value;
        int s_num;

        // Логируем входные данные
        static int log_counter = 0;
        if (++log_counter >= 25) {  // Каждые 0.5 сек при 50Hz
            log_counter = 0;
            RCLCPP_INFO(this->get_logger(), 
                "Input joints: R1=[%.3f,%.3f,%.3f] L1=[%.3f,%.3f,%.3f]",
                legs_jnts->joints_state[0].joint[0], legs_jnts->joints_state[0].joint[1], legs_jnts->joints_state[0].joint[2],
                legs_jnts->joints_state[3].joint[0], legs_jnts->joints_state[3].joint[1], legs_jnts->joints_state[3].joint[2]);
        }

        for (int i = 0; i < num_legs_; i++) {
            for (int j = 0; j < num_joints_; j++) {
                s_num = i * 3 + j;
                target_value = 127 + rotation_direction[s_num] * legs_jnts->joints_state[i].joint[j] * limit_coef_;
                // Clamp to valid range
                target_value = std::max(0.0f, std::min(254.0f, target_value));
                maestro_->setTargetMSS(s_num, static_cast<unsigned char>(target_value));
                // Небольшая задержка между командами для стабильности шины
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }
    }

    Polstro::SerialInterface* maestro_;
    double joint_lower_limit_, joint_upper_limit_, limit_coef_;
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
