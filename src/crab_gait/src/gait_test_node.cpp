// Тестовая нода для проверки походки без джойстика
// Автоматически встаёт и начинает идти вперёд

#include <rclcpp/rclcpp.hpp>
#include <crab_msgs/msg/body_command.hpp>
#include <crab_msgs/msg/gait_command.hpp>

using namespace std::chrono_literals;

class GaitTestNode : public rclcpp::Node {
public:
    GaitTestNode() : Node("gait_test_node"), state_(0) {
        body_cmd_pub_ = this->create_publisher<crab_msgs::msg::BodyCommand>("/teleop/body_command", 1);
        gait_cmd_pub_ = this->create_publisher<crab_msgs::msg::GaitCommand>("/teleop/gait_control", 1);
        
        timer_ = this->create_wall_timer(100ms, std::bind(&GaitTestNode::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Gait test node started");
        RCLCPP_INFO(this->get_logger(), "Sequence: STAND_UP -> wait 3s -> WALK FORWARD -> 10s -> STOP -> SEAT_DOWN");
    }

private:
    void timerCallback() {
        tick_++;
        
        switch (state_) {
            case 0:  // Ждём 1 секунду перед стартом
                if (tick_ >= 10) {
                    RCLCPP_INFO(this->get_logger(), "Sending STAND_UP command...");
                    crab_msgs::msg::BodyCommand cmd;
                    cmd.cmd = crab_msgs::msg::BodyCommand::STAND_UP_CMD;
                    body_cmd_pub_->publish(cmd);
                    state_ = 1;
                    tick_ = 0;
                }
                break;
                
            case 1:  // Ждём 3 секунды после вставания
                if (tick_ >= 30) {
                    RCLCPP_INFO(this->get_logger(), "Starting RIPPLE gait - walking forward...");
                    state_ = 2;
                    tick_ = 0;
                }
                break;
                
            case 2:  // Идём вперёд 10 секунд
                {
                    crab_msgs::msg::GaitCommand gait;
                    gait.cmd = crab_msgs::msg::GaitCommand::RUNRIPPLE;
                    gait.fi = 0.0;      // Вперёд
                    gait.scale = 0.5;   // 50% скорости
                    gait.alpha = 0.0;   // Без поворота
                    gait_cmd_pub_->publish(gait);
                    
                    if (tick_ % 10 == 0) {
                        RCLCPP_INFO(this->get_logger(), "Walking... %d/100", tick_);
                    }
                    
                    if (tick_ >= 100) {
                        RCLCPP_INFO(this->get_logger(), "Stopping gait...");
                        state_ = 3;
                        tick_ = 0;
                    }
                }
                break;
                
            case 3:  // Останавливаем походку
                {
                    crab_msgs::msg::GaitCommand gait;
                    gait.cmd = crab_msgs::msg::GaitCommand::STOP;
                    gait_cmd_pub_->publish(gait);
                    
                    if (tick_ >= 20) {
                        RCLCPP_INFO(this->get_logger(), "Sending SEAT_DOWN command...");
                        crab_msgs::msg::BodyCommand cmd;
                        cmd.cmd = crab_msgs::msg::BodyCommand::SEAT_DOWN_CMD;
                        body_cmd_pub_->publish(cmd);
                        state_ = 4;
                        tick_ = 0;
                    }
                }
                break;
                
            case 4:  // Готово
                if (tick_ == 1) {
                    RCLCPP_INFO(this->get_logger(), "Test complete!");
                }
                break;
        }
    }

    rclcpp::Publisher<crab_msgs::msg::BodyCommand>::SharedPtr body_cmd_pub_;
    rclcpp::Publisher<crab_msgs::msg::GaitCommand>::SharedPtr gait_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int state_ = 0;
    int tick_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GaitTestNode>());
    rclcpp::shutdown();
    return 0;
}
