#include <rclcpp/rclcpp.hpp>
#include <crab_msgs/msg/leg_joints_state.hpp>
#include "PolstroSerialInterface.h"

class MaestroLegSubscriber : public rclcpp::Node {
public:
    MaestroLegSubscriber() : Node("maestro_leg_sub"), maestro_(nullptr) {
        this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 9600);

        std::string port_name = this->get_parameter("port_name").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        maestro_ = Polstro::SerialInterface::createSerialInterface(port_name, baud_rate);

        sub_ = this->create_subscription<crab_msgs::msg::LegJointsState>(
            "leg_states", 1,
            std::bind(&MaestroLegSubscriber::chatterLegState, this, std::placeholders::_1));
    }

private:
    void chatterLegState(const crab_msgs::msg::LegJointsState::SharedPtr state) {
        float target_value;

        target_value = 6016.0f - (state->joint[0] * 2486.624f);
        maestro_->setTarget(9, static_cast<unsigned short>(target_value));
        RCLCPP_INFO(this->get_logger(), "Servo %d: [%d]", 0, static_cast<int>(target_value));

        target_value = 6016.0f + (state->joint[1] * 2486.624f);
        maestro_->setTarget(10, static_cast<unsigned short>(target_value));
        RCLCPP_INFO(this->get_logger(), "Servo %d: [%d]", 1, static_cast<int>(target_value));

        target_value = 6016.0f - (state->joint[2] * 2486.624f);
        maestro_->setTarget(11, static_cast<unsigned short>(target_value));
        RCLCPP_INFO(this->get_logger(), "Servo %d: [%d]", 2, static_cast<int>(target_value));
    }

    Polstro::SerialInterface* maestro_;
    rclcpp::Subscription<crab_msgs::msg::LegJointsState>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MaestroLegSubscriber>());
    rclcpp::shutdown();
    return 0;
}
