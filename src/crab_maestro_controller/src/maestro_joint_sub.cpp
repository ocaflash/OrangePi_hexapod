#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "PolstroSerialInterface.h"

class MaestroJointSubscriber : public rclcpp::Node {
public:
    MaestroJointSubscriber() : Node("maestro_joint_sub"), maestro_(nullptr) {
        this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 9600);

        std::string port_name = this->get_parameter("port_name").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        maestro_ = Polstro::SerialInterface::createSerialInterface(port_name, baud_rate);

        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 100,
            std::bind(&MaestroJointSubscriber::chatterJointState, this, std::placeholders::_1));
    }

private:
    void chatterJointState(const sensor_msgs::msg::JointState::SharedPtr state) {
        float target_value;
        for (size_t i = 0; i < 3 && i < state->position.size(); i++) {
            target_value = 6016.0f + (state->position[i] * 2486.624f);
            maestro_->setTarget(static_cast<unsigned char>(i), static_cast<unsigned short>(target_value));
            RCLCPP_INFO(this->get_logger(), "Servo %zu: [%d]", i, static_cast<int>(target_value));
        }
    }

    Polstro::SerialInterface* maestro_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MaestroJointSubscriber>());
    rclcpp::shutdown();
    return 0;
}
