#ifndef BODY_KINEMATICS_HPP_
#define BODY_KINEMATICS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <crab_msgs/srv/get_leg_ik_solver.hpp>
#include <crab_msgs/msg/legs_joints_state.hpp>
#include <crab_msgs/msg/body_state.hpp>
#include <crab_msgs/msg/body_command.hpp>

#define NUM_LEGS 6
#define NUM_JOINTS 3

class BodyKinematics : public rclcpp::Node {
public:
    BodyKinematics();
    bool init();

private:
    std::string root_name_, tip_name_;
    std::vector<KDL::Frame> frames_;
    crab_msgs::msg::BodyState bs_;
    crab_msgs::msg::LegsJointsState legs_;
    static constexpr unsigned int num_joints_ = NUM_JOINTS;
    static constexpr unsigned int num_legs_ = NUM_LEGS;
    double z_;

    KDL::Rotation rotation_;
    KDL::Frame tibia_foot_frame_, femur_frame_;
    KDL::Vector offset_vector_, rotate_correction_, final_vector_[NUM_LEGS];

    rclcpp::Client<crab_msgs::srv::GetLegIKSolver>::SharedPtr client_;
    rclcpp::Publisher<crab_msgs::msg::LegsJointsState>::SharedPtr joints_pub_;
    rclcpp::Subscription<crab_msgs::msg::BodyState>::SharedPtr body_move_sub_;
    rclcpp::Subscription<crab_msgs::msg::BodyCommand>::SharedPtr body_cmd_sub_;
    rclcpp::TimerBase::SharedPtr motion_timer_;
    
    bool stand_up_active_;
    bool seat_down_active_;

    bool loadModel(const std::string& xml);
    bool calculateKinematics(crab_msgs::msg::BodyState* body_ptr);
    bool callService(KDL::Vector* vector);
    void teleopBodyMove(const crab_msgs::msg::BodyState::SharedPtr body_state);
    void teleopBodyCmd(const crab_msgs::msg::BodyCommand::SharedPtr body_cmd);
    void motionTimerCallback();
};

#endif /* BODY_KINEMATICS_HPP_ */
