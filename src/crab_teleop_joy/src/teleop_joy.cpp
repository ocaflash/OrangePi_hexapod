#include "teleop_joy.hpp"
#include <cmath>
#include <algorithm>
#include <thread>

using namespace std::chrono_literals;

// Deadzone threshold for joystick axes
constexpr float DEADZONE = 0.15f;

// Apply deadzone to axis value
inline float applyDeadzone(float value) {
    return (std::abs(value) < DEADZONE) ? 0.0f : value;
}

// Safe access helpers for varying controller mappings
static inline int getButton(const sensor_msgs::msg::Joy::SharedPtr& joy, size_t idx) {
    return (joy && joy->buttons.size() > idx) ? joy->buttons[idx] : 0;
}

static inline float getAxis(const sensor_msgs::msg::Joy::SharedPtr& joy, size_t idx) {
    return (joy && joy->axes.size() > idx) ? joy->axes[idx] : 0.0f;
}

TeleopJoy::TeleopJoy() : Node("teleop_joy"), start_flag_(false), gait_flag_(false), 
                         imu_flag_(false), gyro_button_pressed_(false),
                         ds4_gyro_x_(0), ds4_gyro_y_(0), ds4_gyro_z_(0) {
    this->declare_parameter<double>("clearance", 0.045);
    this->declare_parameter<double>("seat_height", 0.016);
    this->declare_parameter<double>("default_leg_radius", 0.11);
    z_ = this->get_parameter("clearance").as_double();
    seat_height_ = this->get_parameter("seat_height").as_double();
    default_leg_radius_ = this->get_parameter("default_leg_radius").as_double();

    body_state_.leg_radius = default_leg_radius_;
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
    RCLCPP_INFO(this->get_logger(), "Controls: OPTIONS=stand/sit, Left stick=walk, Right stick=turn/scale");
    RCLCPP_INFO(this->get_logger(), "Hold L1=body RPY, hold R1=body XYZ offsets, hold Square=gyro control");
    RCLCPP_INFO(this->get_logger(), "Hold Square button and tilt controller for gyro control");
}

void TeleopJoy::ds4ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu) {
    ds4_gyro_x_ = imu->angular_velocity.x;
    ds4_gyro_y_ = imu->angular_velocity.y;
    ds4_gyro_z_ = imu->angular_velocity.z;
    
    // If gyro button is pressed and robot is standing, use gyro for body control
    if (gyro_button_pressed_ && start_flag_) {
        // Scale factors reduced for smaller amplitude
        // Max tilt ~0.15 rad (~8.5 degrees)
        body_state_.roll = std::clamp(ds4_gyro_x_ * 0.02, -0.15, 0.15);
        body_state_.pitch = std::clamp(-ds4_gyro_y_ * 0.02, -0.15, 0.15);
        move_body_pub_->publish(body_state_);
    }
}

void TeleopJoy::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
    const int btn_start = getButton(joy, button_start_);
    const int btn_imu = getButton(joy, button_imu_);
    const int btn_gait_switch = getButton(joy, button_gait_switch_);
    const int btn_l1 = getButton(joy, button_left_shift_);
    const int btn_r1 = getButton(joy, button_right_shift_);
    const int btn_gyro = getButton(joy, button_gyro_control_);

    const float ax_lx = getAxis(joy, axis_body_roll_);
    const float ax_ly = getAxis(joy, axis_body_pitch_);
    const float ax_rx = getAxis(joy, axis_body_yaw_);
    const float ax_ry = getAxis(joy, axis_body_z_off_);

    // Rising-edge detection (act once per press, not while held) + time debounce
    const bool start_pressed = (btn_start != 0) && (prev_btn_start_ == 0);
    const bool imu_pressed = (btn_imu != 0) && (prev_btn_imu_ == 0);
    const bool gait_switch_pressed = (btn_gait_switch != 0) && (prev_btn_gait_switch_ == 0);
    const auto now = this->now();
    const auto debounce = rclcpp::Duration::from_seconds(0.35);

    if (start_pressed && !imu_flag_ && (now - last_start_toggle_time_) > debounce) {
        last_start_toggle_time_ = now;

        // IMPORTANT: OPTIONS should NEVER start walking. Stop gait explicitly on mode change.
        gait_command_.cmd = crab_msgs::msg::GaitCommand::STOP;
        gait_command_.scale = 0.0;
        gait_command_.alpha = 0.0;
        gait_command_.fi = 0.0;
        gait_cmd_pub_->publish(gait_command_);

        if (!start_flag_) {
            start_flag_ = true;
            body_command_.cmd = crab_msgs::msg::BodyCommand::STAND_UP_CMD;
            body_cmd_pub_->publish(body_command_);
            // Ensure teleop body_state does not keep "seated" Z / leg_radius after standing up
            body_state_.z = -z_;
            // IMPORTANT: use nominal leg radius for standing/walking to keep IK reachable
            body_state_.leg_radius = default_leg_radius_;
            move_body_pub_->publish(body_state_);
        } else {
            start_flag_ = false;
            body_command_.cmd = crab_msgs::msg::BodyCommand::SEAT_DOWN_CMD;
            body_cmd_pub_->publish(body_command_);
            // Switch teleop state to seated posture
            body_state_.z = -seat_height_;
            body_state_.roll = 0.0;
            body_state_.pitch = 0.0;
            body_state_.yaw = 0.0;
            move_body_pub_->publish(body_state_);
        }
    }

    if (imu_pressed && !start_flag_ && (now - last_imu_toggle_time_) > debounce) {
        last_imu_toggle_time_ = now;
        if (!imu_flag_) {
            imu_flag_ = true;
            body_command_.cmd = crab_msgs::msg::BodyCommand::IMU_START_CMD;
            body_cmd_pub_->publish(body_command_);
        } else {
            imu_flag_ = false;
            body_command_.cmd = crab_msgs::msg::BodyCommand::IMU_STOP_CMD;
            body_cmd_pub_->publish(body_command_);
        }
    }

    if (gait_switch_pressed && (now - last_gait_toggle_time_) > debounce) {
        last_gait_toggle_time_ = now;
        if (!gait_flag_) {
            gait_flag_ = true;
            gait_command_.cmd = crab_msgs::msg::GaitCommand::STOP;
        } else {
            gait_flag_ = false;
            gait_command_.cmd = crab_msgs::msg::GaitCommand::STOP;
        }
        gait_cmd_pub_->publish(gait_command_);
    }

    if (start_flag_) {
        // RPY Signal via sticks (L1 held)
        if (btn_l1) {
            body_state_.roll = 0.25 * ax_lx;
            body_state_.pitch = -0.25 * ax_ly;
            body_state_.yaw = -0.28 * ax_rx;
            move_body_pub_->publish(body_state_);
        }
        // RPY Signal via controller gyroscope (Square held)
        // Gyro data comes from ds4_imu_publisher via /ds4/imu topic
        gyro_button_pressed_ = (btn_gyro != 0);
        // Note: actual gyro control happens in ds4ImuCallback when button is pressed
        // Offset Signal (R1 held)
        if (btn_r1) {
            body_state_.y = -0.05 * ax_lx;
            body_state_.x = -0.05 * ax_ly;
            if (ax_ry < 0) {
                body_state_.z = -0.03 * ax_ry - z_;
            } else {
                body_state_.z = -0.1 * ax_ry - z_;
            }
            move_body_pub_->publish(body_state_);
        }
        // Gait Signals
        if (!btn_l1 && !btn_r1) {
            // Apply deadzone to all axes
            float fi_x = applyDeadzone(getAxis(joy, axis_fi_x_));
            float fi_y = applyDeadzone(getAxis(joy, axis_fi_y_));
            float alpha = applyDeadzone(getAxis(joy, axis_alpha_));
            float scale = applyDeadzone(getAxis(joy, axis_scale_));

            // Start walking ONLY from left stick (fi_x/fi_y). This prevents "walking on OPTIONS"
            // due to right-stick drift.
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
            } else {
                // All sticks in neutral - PAUSE
                gait_command_.cmd = crab_msgs::msg::GaitCommand::PAUSE;
                gait_command_.scale = 0.0;
                gait_command_.alpha = 0.0;
            }
            gait_cmd_pub_->publish(gait_command_);
        }
    } else {
        // Leg radius adjustment when seated (L2 trigger pressed - use axis, not button)
        // L2 axis: 1.0 = not pressed, -1.0 = fully pressed
        bool l2_pressed = (joy->axes.size() > DS4_AXIS_L2) && (joy->axes[DS4_AXIS_L2] < 0.5);

        // Helpful hint: sticks won't walk while seated (start_flag_ == false)
        const bool sticks_moved =
            (std::abs(ax_lx) > 0.2f) || (std::abs(ax_ly) > 0.2f) ||
            (std::abs(ax_rx) > 0.2f) || (std::abs(ax_ry) > 0.2f);
        if (sticks_moved && !l2_pressed) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Robot is SEATED. Press OPTIONS to stand up; walking commands are ignored while seated. "
                "(Deadzone=%.2f)", DEADZONE);
        }

        if (l2_pressed && !imu_flag_) {
            body_state_.z = -seat_height_;  // Seated position
            // Keep radius changes conservative to avoid IK failures during low Z
            body_state_.leg_radius = std::clamp(0.04 * ax_rx + default_leg_radius_, 0.10, 0.12);
            // Keep seated posture stable (avoid confusing stale RPY output)
            body_state_.roll = 0.0;
            body_state_.pitch = 0.0;
            body_state_.yaw = 0.0;
            move_body_pub_->publish(body_state_);
            RCLCPP_DEBUG(this->get_logger(), "L2 pressed: leg_radius=%.3f", body_state_.leg_radius);
        }
        // Only publish STOP once when transitioning to stopped state
        if (gait_command_.cmd != crab_msgs::msg::GaitCommand::STOP) {
            gait_command_.cmd = crab_msgs::msg::GaitCommand::STOP;
            gait_cmd_pub_->publish(gait_command_);
        }
    }

    // Update previous button states at end of callback
    prev_btn_start_ = btn_start;
    prev_btn_imu_ = btn_imu;
    prev_btn_gait_switch_ = btn_gait_switch;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopJoy>());
    rclcpp::shutdown();
    return 0;
}
