#include <rclcpp/rclcpp.hpp>
#include <crab_msgs/msg/legs_joints_state.hpp>
#include <algorithm>
#include "PolstroSerialInterface.h"

// Направление вращения для каждого канала
// Правые ноги: coxa(+), femur(-), tibia(+)
// Левые ноги: coxa(+), femur(+), tibia(-)
const int rotation_direction[18] = {
    1, -1, 1,   // R1: ch 0,1,2
    1, -1, 1,   // R2: ch 3,4,5
    1, -1, 1,   // R3: ch 6,7,8
    1, 1, -1,   // L1: ch 9,10,11
    1, 1, -1,   // L2: ch 12,13,14
    1, 1, -1    // L3: ch 15,16,17
};

class MaestroController : public rclcpp::Node {
public:
    MaestroController() : Node("maestro_servo_node") {
        this->declare_parameter<double>("joint_lower_limit", -1.570796327);
        this->declare_parameter<double>("joint_upper_limit", 1.570796327);
        this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 115200);

        joint_lower_limit_ = this->get_parameter("joint_lower_limit").as_double();
        joint_upper_limit_ = this->get_parameter("joint_upper_limit").as_double();
        limit_coef_ = 127 / ((joint_upper_limit_ - joint_lower_limit_) / 2);

        std::string port_name = this->get_parameter("port_name").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        maestro_ = Polstro::SerialInterface::createSerialInterface(port_name, baud_rate);

        if (maestro_ && maestro_->isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Maestro connected on %s @ %d baud", 
                        port_name.c_str(), baud_rate);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open Maestro on %s", port_name.c_str());
        }

        sub_ = this->create_subscription<crab_msgs::msg::LegsJointsState>(
            "joints_to_controller", 1,
            std::bind(&MaestroController::chatterLegsState, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Maestro servo controller ready");
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

        for (int i = 0; i < num_legs_; i++) {
            for (int j = 0; j < num_joints_; j++) {
                s_num = i * 3 + j;
                target_value = 127 + rotation_direction[s_num] * legs_jnts->joints_state[i].joint[j] * limit_coef_;
                target_value = std::max(0.0f, std::min(254.0f, target_value));
                maestro_->setTargetMSS(s_num, static_cast<unsigned char>(target_value));
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
