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

    RCLCPP_INFO(this->get_logger(), "Teleop started. Press OPTIONS to stand up.");
}

void TeleopJoy::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
    // Логируем первое сообщение
    static bool first_msg = true;
    if (first_msg) {
        RCLCPP_INFO(this->get_logger(), "Joy connected: %zu axes, %zu buttons", 
                    joy->axes.size(), joy->buttons.size());
        first_msg = false;
    }

    // Безопасный доступ к кнопкам и осям
    auto getButton = [&](size_t idx) -> int {
        return (joy->buttons.size() > idx) ? joy->buttons[idx] : 0;
    };
    auto getAxis = [&](size_t idx) -> float {
        return (joy->axes.size() > idx) ? joy->axes[idx] : 0.0f;
    };

    // OPTIONS - встать/сесть
    if (getButton(button_start_) && !imu_flag_) {
        if (!start_flag_) {
            start_flag_ = true;
            body_command_.cmd = crab_msgs::msg::BodyCommand::STAND_UP_CMD;
            body_cmd_pub_->publish(body_command_);
            RCLCPP_INFO(this->get_logger(), "STAND_UP command sent");
        } else {
            start_flag_ = false;
            body_command_.cmd = crab_msgs::msg::BodyCommand::SEAT_DOWN_CMD;
            body_cmd_pub_->publish(body_command_);
            RCLCPP_INFO(this->get_logger(), "SEAT_DOWN command sent");
        }
        std::this_thread::sleep_for(1000ms);
    }

    // CROSS - IMU режим
    if (getButton(button_imu_) && !start_flag_) {
        if (!imu_flag_) {
            imu_flag_ = true;
            body_command_.cmd = crab_msgs::msg::BodyCommand::IMU_START_CMD;
            body_cmd_pub_->publish(body_command_);
        } else {
            imu_flag_ = false;
            body_command_.cmd = crab_msgs::msg::BodyCommand::IMU_STOP_CMD;
            body_cmd_pub_->publish(body_command_);
        }
        std::this_thread::sleep_for(1000ms);
    }

    // TRIANGLE - переключение походки
    if (getButton(button_gait_switch_)) {
        gait_flag_ = !gait_flag_;
        gait_command_.cmd = crab_msgs::msg::GaitCommand::STOP;
        gait_cmd_pub_->publish(gait_command_);
        std::this_thread::sleep_for(500ms);
    }

    if (start_flag_) {
        // L1 - управление RPY
        if (getButton(button_left_shift_)) {
            body_state_.roll = 0.25 * getAxis(axis_body_roll_);
            body_state_.pitch = -0.25 * getAxis(axis_body_pitch_);
            body_state_.yaw = -0.28 * getAxis(axis_body_yaw_);
            move_body_pub_->publish(body_state_);
        }
        // R1 - управление смещением
        if (getButton(button_right_shift_)) {
            body_state_.y = -0.05 * getAxis(axis_body_y_off_);
            body_state_.x = -0.05 * getAxis(axis_body_x_off_);
            if (getAxis(axis_body_z_off_) < 0) {
                body_state_.z = -0.03 * getAxis(axis_body_z_off_) - z_;
            } else {
                body_state_.z = -0.1 * getAxis(axis_body_z_off_) - z_;
            }
            move_body_pub_->publish(body_state_);
        }
        // Походка - когда L1 и R1 не нажаты
        if (!getButton(button_left_shift_) && !getButton(button_right_shift_)) {
            float fi_x = getAxis(axis_fi_x_);
            float fi_y = getAxis(axis_fi_y_);
            float alpha = getAxis(axis_alpha_);
            float scale = getAxis(axis_scale_);

            // Левый стик - направление и скорость
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
            }

            // Правый стик - поворот на месте
            if (alpha != 0 || scale != 0) {
                if (!gait_flag_) {
                    gait_command_.cmd = crab_msgs::msg::GaitCommand::RUNRIPPLE;
                } else {
                    gait_command_.cmd = crab_msgs::msg::GaitCommand::RUNTRIPOD;
                }
                gait_command_.fi = (scale > 0) ? 0 : 3.14;
                gait_command_.scale = scale;
                if (gait_command_.scale < 0) gait_command_.scale *= -1;
                gait_command_.alpha = ((alpha > 0) ? 1 : -1) * 0.06 * (1 - gait_command_.scale) + 0.11 * alpha;
            }

            // Все стики в нейтрали - пауза
            if (!alpha && !scale && !fi_x && !fi_y) {
                gait_command_.cmd = crab_msgs::msg::GaitCommand::PAUSE;
            }
            gait_cmd_pub_->publish(gait_command_);
        }
    } else {
        // Робот сидит - регулировка leg_radius
        if (getButton(button_right_shift_2_) && !imu_flag_) {
            body_state_.z = -0.01;
            body_state_.leg_radius = 0.06 * getAxis(axis_body_yaw_) + 0.11;
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
