#ifndef GAIT_KINEMATICS_HPP_
#define GAIT_KINEMATICS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <crab_msgs/srv/get_leg_ik_solver.hpp>
#include <crab_msgs/msg/legs_joints_state.hpp>
#include <crab_msgs/msg/gait_command.hpp>
#include <crab_msgs/msg/leg_position_state.hpp>
#include "gait.hpp"

#define NUM_LEGS 6
#define NUM_JOINTS 3

class GaitKinematics : public rclcpp::Node {
public:
    GaitKinematics();
    bool init();
    void gaitGenerator();

private:
    std::string root_name_, tip_name_;
    std::vector<KDL::Frame> frames_;
    double fi_;
    crab_msgs::msg::LegsJointsState legs_;
    crab_msgs::msg::GaitCommand gait_command_;
    static constexpr unsigned int num_joints_ = NUM_JOINTS;
    static constexpr unsigned int num_legs_ = NUM_LEGS;
    double trap_low_r_, trap_high_r_, trap_h_, trap_z_;
    double d_ripple_, d_tripod_;
    double leg_radius_;
    double path_tolerance_, rounded_radius_;

    rclcpp::Client<crab_msgs::srv::GetLegIKSolver>::SharedPtr client_;
    rclcpp::Publisher<crab_msgs::msg::LegsJointsState>::SharedPtr joints_pub_;
    rclcpp::Subscription<crab_msgs::msg::GaitCommand>::SharedPtr gait_control_sub_;

    bool loadModel(const std::string& xml);
    bool callService(KDL::Vector* vector);
    void teleopGaitCtrl(const crab_msgs::msg::GaitCommand::SharedPtr gait_cmd);
};

#endif /* GAIT_KINEMATICS_HPP_ */
