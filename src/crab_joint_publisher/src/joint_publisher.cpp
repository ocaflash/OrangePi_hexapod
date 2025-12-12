#include <rclcpp/rclcpp.hpp>
#include <crab_msgs/msg/legs_joints_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

const std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};
const std::string names[3] = {"coxa_joint", "femur_joint", "tibia_joint"};

class JointPublisher : public rclcpp::Node {
public:
    JointPublisher() : Node("crab_joint_publisher") {
        joint_msg_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("crab_joint_publisher", 1);
        sub_ = this->create_subscription<crab_msgs::msg::LegsJointsState>(
            "joints_to_controller", 1,
            std::bind(&JointPublisher::chatterLegsState, this, std::placeholders::_1));
    }

private:
    void chatterLegsState(const crab_msgs::msg::LegsJointsState::SharedPtr state) {
        sensor_msgs::msg::JointState joint_msg;
        joint_msg.header.stamp = this->now();
        
        std::string joint_name;
        for (int name = 0; name < 3; name++) {
            for (int suf = 0; suf < 6; suf++) {
                joint_name = names[name] + suffixes[suf];
                joint_msg.name.push_back(joint_name);
                joint_msg.position.push_back(state->joints_state[suf].joint[name]);
            }
        }
        joint_msg_pub_->publish(joint_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_msg_pub_;
    rclcpp::Subscription<crab_msgs::msg::LegsJointsState>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPublisher>());
    rclcpp::shutdown();
    return 0;
}
