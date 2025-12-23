#ifndef CONTROLLER_SUB_HPP_
#define CONTROLLER_SUB_HPP_

#include <rclcpp/rclcpp.hpp>
#include <crab_msgs/msg/legs_joints_state.hpp>
#include "PolstroSerialInterface.h"

class Controller : public rclcpp::Node {
public:
    Controller();

private:
    void chatterLegsState(const crab_msgs::msg::LegsJointsState::SharedPtr legs_jnts);

    std::string port_name_;
    Polstro::SerialInterface* maestro_;
    double joint_lower_limit_;
    double joint_upper_limit_;
    double limit_coef_;
    int baud_rate_;

    static constexpr unsigned int num_joints = 3;
    static constexpr unsigned int num_legs = 6;

    rclcpp::Subscription<crab_msgs::msg::LegsJointsState>::SharedPtr sub_;
};

#endif /* CONTROLLER_SUB_HPP_ */
