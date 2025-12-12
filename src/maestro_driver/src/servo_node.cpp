#include <rclcpp/rclcpp.hpp>
#include <crab_msgs/msg/legs_joints_state.hpp>
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
    MaestroController() : Node("maestro_servo_node") {
        this->declare_parameter<double>("joint_lower_limit", -1.570796327);
        this->declare_parameter<double>("joint_upper_limit", 1.570796327);
        this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");

        joint_lower_limit_ = this->get_parameter("joint_lower_limit").as_double();
        joint_upper_limit_ = this->get_parameter("joint_upper_limit").as_double();
        limit_coef_ = 127 / ((joint_upper_limit_ - joint_lower_limit_) / 2);

        std::string port_name = this->get_parameter("port_name").as_string();
        maestro_ = Polstro::SerialInterface::createSerialInterface(port_name, 9600);

        if (maestro_ && maestro_->isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Maestro servo controller connected on %s", port_name.c_str());
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

        for (int i = 0; i < num_legs_; i++) {
            for (int j = 0; j < num_joints_; j++) {
                s_num = i * 3 + j;
                target_value = 127 + rotation_direction[s_num] * legs_jnts->joints_state[i].joint[j] * limit_coef_;
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
