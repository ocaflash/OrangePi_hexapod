#ifndef LEG_IK_SERVICE_HPP_
#define LEG_IK_SERVICE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "chainiksolvervel_pinv.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include "hp_chainiksolverpos_nr_jl.hpp"
#include <crab_msgs/srv/get_leg_ik_solver.hpp>
#include <crab_msgs/msg/leg_joints_state.hpp>
#include <crab_msgs/msg/leg_position_state.hpp>

#define NUM_LEGS 6
#define NUM_JOINTS 3

class LegKinematics : public rclcpp::Node {
public:
    LegKinematics();
    bool init();

private:
    std::string root_name_, tip_name_;
    double joint_lower_limit_, joint_upper_limit_;
    static constexpr unsigned int num_joints_ = NUM_JOINTS;
    static constexpr unsigned int num_legs_ = NUM_LEGS;

    KDL::Chain* chains_ptr_[6];
    KDL::JntArray joint_min_, joint_max_;
    KDL::ChainFkSolverPos_recursive* fk_solver_[6];
    KDL::HP_ChainIkSolverPos_NR_JL* ik_solver_pos_[6];
    KDL::ChainIkSolverVel_pinv* ik_solver_vel_[6];

    rclcpp::Service<crab_msgs::srv::GetLegIKSolver>::SharedPtr ik_service_;

    bool loadModel(const std::string& xml);
    void getLegIKSolver(const std::shared_ptr<crab_msgs::srv::GetLegIKSolver::Request> request,
                        std::shared_ptr<crab_msgs::srv::GetLegIKSolver::Response> response);
};

#endif /* LEG_IK_SERVICE_HPP_ */
