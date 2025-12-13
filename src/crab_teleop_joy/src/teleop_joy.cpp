#include "teleop_joy.hpp"
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

// Deadzone threshold for joystick axes
constexpr float DEADZONE = 0.15f;

// Apply deadzone to axis value
inline float applyDeadzone(float value) {
    return (std::abs(value) < DEADZONE) ? 0.0f : value;
}

TeleopJoy::TeleopJoy() : Node("teleop_joy"), start_flag_(false), gait_flag_(false), 
                         imu_flag_(false), gyro_button_pressed_(false),
                         ds4_gyro_x_(0), ds4_gyro_y_(0), ds4_gyro_z_(0) {
    this->declare_parameter<double>("clearance", 0.045);
    z_ = this->get_parameter("clearance").as_double();

    body_state_.leg_radius = 0.11;
    body_state_.z = -z_;

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 1, std::bind(&TeleopJoy::joyCallback, this, std::placeholders::_1));
    
    // Subscribe to DS4 IMU data from ds4_imu_publisher node
    ds4_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/ds4/imu", 10, std::bind(&TeleopJoy::ds4ImuCallback, this, std::placeholders::_1));
    
    move_body_pub_ = this->create_publisher<crab_msgs::msg::BodyState>("/teleop/move_body", 1);
    body_cmd_pub_ = this->create_publisher<crab_msgs::msg::BodyCommand>("/teleop/body_command", 1);
    gait_cmd_pub_ = this->create_publisher<crab_msgs::msg::GaitCommand>("/teleop/gait_control", 1);

    RCLCPP_INFO(this->get_logger(), "Starting DS4 teleop converter...");
    RCLCPP_INFO(this->get_logger(), "Hold Square button and tilt controller for gyro control");
}

void TeleopJoy::ds4ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu) {
    ds4_gyro_x_ = imu->angular_velocity.x;
    ds4_gyro_y_ = imu->angular_velocity.y;
    ds4_gyro_z_ = imu->angular_velocity.z;
    
    // If gyro button is pressed and robot is standing, use gyro for body control
    if (gyro_button_pressed_ && start_flag_) {
        // Integrate gyro to get orientation (simple approach)
        // Scale factors tuned for natural feel
        body_state_.roll = std::clamp(ds4_gyro_x_ * 0.1, -0.3, 0.3);
        body_state_.pitch = std::clamp(-ds4_gyro_y_ * 0.1, -0.3, 0.3);
        move_body_pub_->publish(body_state_);
    }
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
        // RPY Signal via sticks (L1 held)
        if (joy->buttons[button_left_shift_]) {
            body_state_.roll = 0.25 * joy->axes[axis_body_roll_];
            body_state_.pitch = -0.25 * joy->axes[axis_body_pitch_];
            body_state_.yaw = -0.28 * joy->axes[axis_body_yaw_];
            move_body_pub_->publish(body_state_);
        }
        // RPY Signal via controller gyroscope (Square held)
        // Gyro data comes from ds4_imu_publisher via /ds4/imu topic
        gyro_button_pressed_ = joy->buttons[button_gyro_control_];
        // Note: actual gyro control happens in ds4ImuCallback when button is pressed
        // Offset Signal (R1 held)
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
            // Apply deadzone to all axes
            float fi_x = applyDeadzone(joy->axes[axis_fi_x_]);
            float fi_y = applyDeadzone(joy->axes[axis_fi_y_]);
            float alpha = applyDeadzone(joy->axes[axis_alpha_]);
            float scale = applyDeadzone(joy->axes[axis_scale_]);

            if (fi_x != 0 || fi_y != 0) {
                if (!gait_flag_) {
                    gait_command_.cmd = crab_msgs::msg::GaitCommand::RUNRIPPLE;
                } else {
                    gait_command_.cmd = crab_msgs::msg::GaitCommand::RUNTRIPOD;
                }
                float a = std::pow(fi_x, 2);
                float b = std::pow(fi_y, 2);
                gait_command_.fi = std::atan2(fi_x, fi_y);
                gait_command_.scale = std::pow(a + b, 0.5) > 1 ? 1 : std::pow(a + b, 0.5);
                gait_command_.alpha = 0;
            } else if (alpha != 0 || scale != 0) {
                if (!gait_flag_) {
                    gait_command_.cmd = crab_msgs::msg::GaitCommand::RUNRIPPLE;
                } else {
                    gait_command_.cmd = crab_msgs::msg::GaitCommand::RUNTRIPOD;
                }
                gait_command_.fi = (scale > 0) ? 0 : 3.14;
                gait_command_.scale = std::abs(scale);
                gait_command_.alpha = ((alpha > 0) ? 1 : -1) * 0.06 * (1 - gait_command_.scale) + 0.11 * alpha;
            } else {
                // All sticks in neutral - PAUSE
                gait_command_.cmd = crab_msgs::msg::GaitCommand::PAUSE;
            }
            gait_cmd_pub_->publish(gait_command_);
        }
    } else {
        // Leg radius adjustment when seated (L2 held)
        if (joy->buttons[button_right_shift_2_] && !imu_flag_) {
            body_state_.z = -0.016;  // Seated position (was -0.01 which caused IK errors)
            body_state_.leg_radius = 0.06 * joy->axes[axis_body_yaw_] + 0.11;
            move_body_pub_->publish(body_state_);
        }
        // Only publish STOP once when transitioning to stopped state
        if (gait_command_.cmd != crab_msgs::msg::GaitCommand::STOP) {
            gait_command_.cmd = crab_msgs::msg::GaitCommand::STOP;
            gait_cmd_pub_->publish(gait_command_);
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopJoy>());
    rclcpp::shutdown();
    return 0;
}
