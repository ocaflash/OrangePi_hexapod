#include "teleop_joy.hpp"
#include <cmath>

using namespace std::chrono_literals;

TeleopJoy::TeleopJoy() : Node("teleop_joy"), start_flag_(false), gait_flag_(false), imu_flag_(false) {
    this->declare_parameter<double>("clearance", 0.045);
    z_ = this->get_parameter("clearance").as_double();

    body_state_.leg_radius = 0.11;
    body_state_.z = -z_;

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 1, std::bind(&TeleopJoy::joyCallback, this, std::placeholders::_1));
    move_body_pub_ = this->create_publisher<crab_msgs::msg::BodyState>("/teleop/move_body", 1);
    body_cmd_pub_ = this->create_publisher<crab_msgs::msg::BodyCommand>("/teleop/body_command", 1);
    gait_cmd_pub_ = this->create_publisher<crab_msgs::msg::GaitCommand>("/teleop/gait_control", 1);

    RCLCPP_INFO(this->get_logger(), "Starting PS3 teleop converter, take care of your controller now...");
}

void TeleopJoy::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
    if (joy->buttons[button_start_] && !imu_flag_) {
        if (!start_flag_) {
            start_flag_ = true;
            body_command_.cmd = crab_msgs::msg::BodyCommand::STAND_UP_CMD;
            body_cmd_pub_->publish(body_command_);
        } else {
            start_flag_ = false;
            body_command_.cmd = crab_msgs::msg::BodyCommand::SEAT_DOWN_CMD;
            body_cmd_pub_->publish(body_command_);
        }
        std::this_thread::sleep_for(1s);
    }

    if (joy->buttons[button_imu_] && !start_flag_) {
        if (!imu_flag_) {
            imu_flag_ = true;
            body_command_.cmd = crab_msgs::msg::BodyCommand::IMU_START_CMD;
            body_cmd_pub_->publish(body_command_);
        } else {
            imu_flag_ = false;
            body_command_.cmd = crab_msgs::msg::BodyCommand::IMU_STOP_CMD;
            body_cmd_pub_->publish(body_command_);
        }
        std::this_thread::sleep_for(1s);
    }

    if (joy->buttons[button_gait_switch_]) {
        if (!gait_flag_) {
            gait_flag_ = true;
            gait_command_.cmd = crab_msgs::msg::GaitCommand::STOP;
        } else {
            gait_flag_ = false;
            gait_command_.cmd = crab_msgs::msg::GaitCommand::STOP;
        }
        gait_cmd_pub_->publish(gait_command_);
        std::this_thread::sleep_for(500ms);
    }

    if (start_flag_) {
        // RPY Signal
        if (joy->buttons[button_left_shift_]) {
            body_state_.roll = 0.25 * joy->axes[axis_body_roll_];
            body_state_.pitch = -0.25 * joy->axes[axis_body_pitch_];
            body_state_.yaw = -0.28 * joy->axes[axis_body_yaw_];
            move_body_pub_->publish(body_state_);
        }
        // Offset Signal
        if (joy->buttons[button_right_shift_]) {
            body_state_.y = -0.05 * joy->axes[axis_body_y_off_];
            body_state_.x = -0.05 * joy->axes[axis_body_x_off_];
            if (joy->axes[axis_body_z_off_] < 0) {
                body_state_.z = -0.03 * joy->axes[axis_body_z_off_] - z_;
            } else {
                body_state_.z = -0.1 * joy->axes[axis_body_z_off_] - z_;
            }
            move_body_pub_->publish(body_state_);
        }
        // Gait Signals
        if (!joy->buttons[button_left_shift_] && !joy->buttons[button_right_shift_]) {
            if (joy->axes[axis_fi_x_] != 0 || joy->axes[axis_fi_y_] != 0) {
                if (!gait_flag_) {
                    gait_command_.cmd = crab_msgs::msg::GaitCommand::RUNRIPPLE;
                } else {
                    gait_command_.cmd = crab_msgs::msg::GaitCommand::RUNTRIPOD;
                }
                float a = std::pow(joy->axes[axis_fi_x_], 2);
                float b = std::pow(joy->axes[axis_fi_y_], 2);
                gait_command_.fi = std::atan2(joy->axes[axis_fi_x_], joy->axes[axis_fi_y_]);
                gait_command_.scale = std::pow(a + b, 0.5) > 1 ? 1 : std::pow(a + b, 0.5);
                gait_command_.alpha = 0;
            }

            if (joy->axes[axis_alpha_] != 0 || joy->axes[axis_scale_] != 0) {
                if (!gait_flag_) {
                    gait_command_.cmd = crab_msgs::msg::GaitCommand::RUNRIPPLE;
                } else {
                    gait_command_.cmd = crab_msgs::msg::GaitCommand::RUNTRIPOD;
                }
                gait_command_.fi = (joy->axes[axis_scale_] > 0) ? 0 : 3.14;
                gait_command_.scale = joy->axes[axis_scale_];
                if (gait_command_.scale < 0) gait_command_.scale *= -1;
                gait_command_.alpha = ((joy->axes[axis_alpha_] > 0) ? 1 : -1) * 0.06 * (1 - gait_command_.scale) + 0.11 * joy->axes[axis_alpha_];
            }

            if (!joy->axes[axis_alpha_] && !joy->axes[axis_scale_] && !joy->axes[axis_fi_x_] && !joy->axes[axis_fi_y_]) {
                gait_command_.cmd = crab_msgs::msg::GaitCommand::PAUSE;
            }
            gait_cmd_pub_->publish(gait_command_);
        }
    } else {
        if (joy->buttons[button_right_shift_2_] && !imu_flag_) {
            body_state_.z = -0.01;
            body_state_.leg_radius = 0.06 * joy->axes[axis_body_yaw_] + 0.11;
            move_body_pub_->publish(body_state_);
        }
        gait_command_.cmd = crab_msgs::msg::GaitCommand::STOP;
        gait_cmd_pub_->publish(gait_command_);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopJoy>());
    rclcpp::shutdown();
    return 0;
}
