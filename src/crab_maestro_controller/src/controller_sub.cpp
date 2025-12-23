#include "controller_sub.hpp"

const int rotation_direction[18] = {
    1, -1, 1,
    1, -1, 1,
    1, -1, 1,
    1, 1, -1,
    1, 1, -1,
    1, 1, -1
};

Controller::Controller() : Node("controller_sub"), maestro_(nullptr) {
    this->declare_parameter<double>("joint_lower_limit", -1.570796327);
    this->declare_parameter<double>("joint_upper_limit", 1.570796327);
    this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
    this->declare_parameter<int>("baud_rate", 9600);

    joint_lower_limit_ = this->get_parameter("joint_lower_limit").as_double();
    joint_upper_limit_ = this->get_parameter("joint_upper_limit").as_double();
    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();

    limit_coef_ = 127.0 / ((joint_upper_limit_ - joint_lower_limit_) / 2.0);

    maestro_ = Polstro::SerialInterface::createSerialInterface(port_name_, baud_rate_);

    sub_ = this->create_subscription<crab_msgs::msg::LegsJointsState>(
        "joints_to_controller", 1,
        std::bind(&Controller::chatterLegsState, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Maestro servo controller is ready...");
}

void Controller::chatterLegsState(const crab_msgs::msg::LegsJointsState::SharedPtr legs_jnts) {
    float target_value;
    int s_num;

    for (int i = 0; i < static_cast<int>(num_legs); i++) {
        for (int j = 0; j < static_cast<int>(num_joints); j++) {
            s_num = i * 3 + j;
            target_value = 127.0f + rotation_direction[s_num] * legs_jnts->joints_state[i].joint[j] * limit_coef_;
            maestro_->setTargetMSS(s_num, static_cast<unsigned char>(target_value));
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
