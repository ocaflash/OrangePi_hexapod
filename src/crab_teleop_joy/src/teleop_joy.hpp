#ifndef TELEOP_JOY_HPP_
#define TELEOP_JOY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <crab_msgs/msg/body_state.hpp>
#include <crab_msgs/msg/body_command.hpp>
#include <crab_msgs/msg/gait_command.hpp>

// PS3 controller button/axis mappings
#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3

class TeleopJoy : public rclcpp::Node {
public:
    TeleopJoy();

private:
    crab_msgs::msg::BodyState body_state_;
    crab_msgs::msg::BodyCommand body_command_;
    crab_msgs::msg::GaitCommand gait_command_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<crab_msgs::msg::BodyState>::SharedPtr move_body_pub_;
    rclcpp::Publisher<crab_msgs::msg::BodyCommand>::SharedPtr body_cmd_pub_;
    rclcpp::Publisher<crab_msgs::msg::GaitCommand>::SharedPtr gait_cmd_pub_;

    double z_;
    bool start_flag_;
    bool gait_flag_;
    bool imu_flag_;

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);

    static constexpr int axis_body_roll_ = PS3_AXIS_STICK_LEFT_LEFTWARDS;
    static constexpr int axis_body_pitch_ = PS3_AXIS_STICK_LEFT_UPWARDS;
    static constexpr int axis_body_yaw_ = PS3_AXIS_STICK_RIGHT_LEFTWARDS;
    static constexpr int axis_body_y_off_ = PS3_AXIS_STICK_LEFT_LEFTWARDS;
    static constexpr int axis_body_x_off_ = PS3_AXIS_STICK_LEFT_UPWARDS;
    static constexpr int axis_body_z_off_ = PS3_AXIS_STICK_RIGHT_UPWARDS;
    static constexpr int button_left_shift_ = PS3_BUTTON_REAR_LEFT_1;
    static constexpr int button_right_shift_ = PS3_BUTTON_REAR_RIGHT_1;
    static constexpr int button_right_shift_2_ = PS3_BUTTON_REAR_LEFT_2;
    static constexpr int button_start_ = PS3_BUTTON_START;
    static constexpr int axis_fi_x_ = PS3_AXIS_STICK_LEFT_LEFTWARDS;
    static constexpr int axis_fi_y_ = PS3_AXIS_STICK_LEFT_UPWARDS;
    static constexpr int button_gait_switch_ = PS3_BUTTON_ACTION_TRIANGLE;
    static constexpr int axis_alpha_ = PS3_AXIS_STICK_RIGHT_LEFTWARDS;
    static constexpr int axis_scale_ = PS3_AXIS_STICK_RIGHT_UPWARDS;
    static constexpr int button_imu_ = PS3_BUTTON_ACTION_CROSS;
};

#endif /* TELEOP_JOY_HPP_ */
