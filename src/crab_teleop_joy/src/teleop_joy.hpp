#ifndef TELEOP_JOY_HPP_
#define TELEOP_JOY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <crab_msgs/msg/body_state.hpp>
#include <crab_msgs/msg/body_command.hpp>
#include <crab_msgs/msg/gait_command.hpp>

// DualShock 4 controller button mappings
#define DS4_BUTTON_CROSS             0   // X
#define DS4_BUTTON_CIRCLE            1   // O
#define DS4_BUTTON_TRIANGLE          2   // Triangle
#define DS4_BUTTON_SQUARE            3   // Square
#define DS4_BUTTON_L1                4
#define DS4_BUTTON_R1                5
#define DS4_BUTTON_L2                6
#define DS4_BUTTON_R2                7
#define DS4_BUTTON_SHARE             8
#define DS4_BUTTON_OPTIONS           9   // Start equivalent
#define DS4_BUTTON_PS                10
#define DS4_BUTTON_L3                11  // Left stick press
#define DS4_BUTTON_R3                12  // Right stick press

// DualShock 4 axis mappings
#define DS4_AXIS_LEFT_STICK_X        0   // Left/Right (-1 to 1)
#define DS4_AXIS_LEFT_STICK_Y        1   // Up/Down (-1 to 1)
#define DS4_AXIS_RIGHT_STICK_X       2   // Left/Right
#define DS4_AXIS_RIGHT_STICK_Y       3   // Up/Down
#define DS4_AXIS_L2                  4   // Trigger (1 to -1)
#define DS4_AXIS_R2                  5   // Trigger
#define DS4_AXIS_DPAD_X              6   // D-pad Left/Right
#define DS4_AXIS_DPAD_Y              7   // D-pad Up/Down

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

    // DualShock 4 mappings
    static constexpr int axis_body_roll_ = DS4_AXIS_LEFT_STICK_X;
    static constexpr int axis_body_pitch_ = DS4_AXIS_LEFT_STICK_Y;
    static constexpr int axis_body_yaw_ = DS4_AXIS_RIGHT_STICK_X;
    static constexpr int axis_body_y_off_ = DS4_AXIS_LEFT_STICK_X;
    static constexpr int axis_body_x_off_ = DS4_AXIS_LEFT_STICK_Y;
    static constexpr int axis_body_z_off_ = DS4_AXIS_RIGHT_STICK_Y;
    static constexpr int button_left_shift_ = DS4_BUTTON_L1;
    static constexpr int button_right_shift_ = DS4_BUTTON_R1;
    static constexpr int button_right_shift_2_ = DS4_BUTTON_L2;
    static constexpr int button_start_ = DS4_BUTTON_OPTIONS;
    static constexpr int axis_fi_x_ = DS4_AXIS_LEFT_STICK_X;
    static constexpr int axis_fi_y_ = DS4_AXIS_LEFT_STICK_Y;
    static constexpr int button_gait_switch_ = DS4_BUTTON_TRIANGLE;
    static constexpr int axis_alpha_ = DS4_AXIS_RIGHT_STICK_X;
    static constexpr int axis_scale_ = DS4_AXIS_RIGHT_STICK_Y;
    static constexpr int button_imu_ = DS4_BUTTON_CROSS;
};

#endif /* TELEOP_JOY_HPP_ */
