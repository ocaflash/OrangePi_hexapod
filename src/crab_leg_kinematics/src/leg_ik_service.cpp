#include "leg_ik_service.hpp"

const static std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};

LegKinematics::LegKinematics() : Node("leg_ik_service") {
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("root_name", "thorax");
    this->declare_parameter<std::string>("tip_name", "tibia_foot");
    this->declare_parameter<double>("joint_lower_limit", -KDL::PI / 2);
    this->declare_parameter<double>("joint_upper_limit", KDL::PI / 2);
    this->declare_parameter<int>("maxIterations", 100);
    this->declare_parameter<double>("epsilon", 1e-3);
}

bool LegKinematics::init() {
    std::string robot_desc_string;

    robot_desc_string = this->get_parameter("robot_description").as_string();
    if (robot_desc_string.empty()) {
        RCLCPP_FATAL(this->get_logger(), "Could not load the xml from parameter: robot_description");
        return false;
    }

    root_name_ = this->get_parameter("root_name").as_string();
    tip_name_ = this->get_parameter("tip_name").as_string();

    if (!loadModel(robot_desc_string)) {
        RCLCPP_FATAL(this->get_logger(), "Could not load models!");
        return false;
    }

    joint_lower_limit_ = this->get_parameter("joint_lower_limit").as_double();
    joint_upper_limit_ = this->get_parameter("joint_upper_limit").as_double();
    joint_min_.resize(num_joints_);
    joint_max_.resize(num_joints_);
    for (unsigned int i = 0; i < num_joints_; i++) {
        joint_min_(i) = joint_lower_limit_;
        joint_max_(i) = joint_upper_limit_;
    }

    int maxIterations = this->get_parameter("maxIterations").as_int();
    double epsilon = this->get_parameter("epsilon").as_double();

    for (unsigned int i = 0; i < num_legs_; i++) {
        fk_solver_[i] = new KDL::ChainFkSolverPos_recursive(*chains_ptr_[i]);
        ik_solver_vel_[i] = new KDL::ChainIkSolverVel_pinv(*chains_ptr_[i]);
        ik_solver_pos_[i] = new KDL::HP_ChainIkSolverPos_NR_JL(*chains_ptr_[i], joint_min_, joint_max_,
                                                                *fk_solver_[i], *ik_solver_vel_[i],
                                                                maxIterations, epsilon);
    }

    RCLCPP_INFO(this->get_logger(), "Advertising service");
    ik_service_ = this->create_service<crab_msgs::srv::GetLegIKSolver>(
        "~/get_ik",
        std::bind(&LegKinematics::getLegIKSolver, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Ready to client's request...");
    return true;
}

bool LegKinematics::loadModel(const std::string& xml) {
    KDL::Tree tree;
    KDL::Chain chain;
    std::string tip_name_result;

    if (!kdl_parser::treeFromString(xml, tree)) {
        RCLCPP_ERROR(this->get_logger(), "Could not initialize tree object");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Construct tree");

    for (int i = 0; i < num_legs_; i++) {
        tip_name_result = tip_name_ + suffixes[i];
        if (!tree.getChain(root_name_, tip_name_result, chain)) {
            RCLCPP_ERROR(this->get_logger(), "Could not initialize chain_%s object", suffixes[i].c_str());
            return false;
        }
        chains_ptr_[i] = new KDL::Chain(chain);
    }
    RCLCPP_INFO(this->get_logger(), "Construct chains");

    return true;
}

void LegKinematics::getLegIKSolver(const std::shared_ptr<crab_msgs::srv::GetLegIKSolver::Request> request,
                                    std::shared_ptr<crab_msgs::srv::GetLegIKSolver::Response> response) {
    crab_msgs::msg::LegPositionState leg_dest_pos;
    response->target_joints.clear();

    for (size_t i = 0; i < request->leg_number.size(); i++) {
        leg_dest_pos = request->target_position[i];
        KDL::JntArray jnt_pos_in(num_joints_);
        KDL::JntArray jnt_pos_out(num_joints_);

        for (unsigned int j = 0; j < num_joints_; j++) {
            jnt_pos_in(j) = request->current_joints[i].joint[j];
        }
        KDL::Frame F_dest(KDL::Vector(leg_dest_pos.x, leg_dest_pos.y, leg_dest_pos.z));
        
        RCLCPP_DEBUG(this->get_logger(), "IK request leg %d: x=%.4f y=%.4f z=%.4f", 
                     request->leg_number[i], leg_dest_pos.x, leg_dest_pos.y, leg_dest_pos.z);

        int ik_valid = ik_solver_pos_[request->leg_number[i]]->CartToJnt(jnt_pos_in, F_dest, jnt_pos_out);

        if (ik_valid >= 0) {
            crab_msgs::msg::LegJointsState jnt_buf;
            for (unsigned int j = 0; j < num_joints_; j++) {
                jnt_buf.joint[j] = jnt_pos_out(j);
            }
            response->target_joints.push_back(jnt_buf);
            response->error_codes = crab_msgs::srv::GetLegIKSolver::Response::IK_FOUND;
            RCLCPP_DEBUG(this->get_logger(), "IK Solution for leg%s found", suffixes[request->leg_number[i]].c_str());
        } else {
            response->error_codes = crab_msgs::srv::GetLegIKSolver::Response::IK_NOT_FOUND;
            RCLCPP_ERROR(this->get_logger(), "An IK solution could not be found");
            return;
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LegKinematics>();
    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "Could not initialize kinematics node");
        return -1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
