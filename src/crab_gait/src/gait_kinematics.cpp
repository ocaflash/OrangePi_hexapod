#include "gait_kinematics.hpp"

using namespace std::chrono_literals;

const static std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};

GaitKinematics::GaitKinematics() : Node("gait_kinematics") {
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("root_name_body", "leg_center");
    this->declare_parameter<std::string>("tip_name_body", "coxa");
    this->declare_parameter<double>("trapezoid_low_radius", 0.03);
    this->declare_parameter<double>("trapezoid_high_radius", 0.023);
    this->declare_parameter<double>("trapezoid_h", 0.02);
    this->declare_parameter<double>("clearance", 0.045);
    this->declare_parameter<double>("duration_ripple", 1.5);
    this->declare_parameter<double>("duration_tripod", 1.0);
    this->declare_parameter<double>("leg_radius", 0.11);
    this->declare_parameter<double>("path_tolerance", 0.005);
    this->declare_parameter<double>("rounded_radius", 0.02);

    gait_command_.cmd = crab_msgs::msg::GaitCommand::STOP;
}

bool GaitKinematics::init() {
    std::string robot_desc_string;

    robot_desc_string = this->get_parameter("robot_description").as_string();
    if (robot_desc_string.empty()) {
        RCLCPP_FATAL(this->get_logger(), "Could not load the xml from parameter: robot_description");
        return false;
    }

    root_name_ = this->get_parameter("root_name_body").as_string();
    tip_name_ = this->get_parameter("tip_name_body").as_string();

    trap_low_r_ = this->get_parameter("trapezoid_low_radius").as_double();
    trap_high_r_ = this->get_parameter("trapezoid_high_radius").as_double();
    trap_h_ = this->get_parameter("trapezoid_h").as_double();
    trap_z_ = this->get_parameter("clearance").as_double();
    d_ripple_ = this->get_parameter("duration_ripple").as_double();
    d_tripod_ = this->get_parameter("duration_tripod").as_double();
    leg_radius_ = this->get_parameter("leg_radius").as_double();
    path_tolerance_ = this->get_parameter("path_tolerance").as_double();
    rounded_radius_ = this->get_parameter("rounded_radius").as_double();

    if (!loadModel(robot_desc_string)) {
        RCLCPP_FATAL(this->get_logger(), "Could not load models!");
        return false;
    }

    client_ = this->create_client<crab_msgs::srv::GetLegIKSolver>("/crab_leg_kinematics/get_ik");
    joints_pub_ = this->create_publisher<crab_msgs::msg::LegsJointsState>("joints_to_controller", 1);
    gait_control_sub_ = this->create_subscription<crab_msgs::msg::GaitCommand>(
        "/teleop/gait_control", 1,
        std::bind(&GaitKinematics::teleopGaitCtrl, this, std::placeholders::_1));

    return true;
}

void GaitKinematics::gaitGenerator() {
    KDL::Vector* final_vector;
    Gait gait;
    gait.setTrapezoid(trap_low_r_, trap_high_r_, trap_h_, trap_z_, path_tolerance_, rounded_radius_);

    RCLCPP_INFO(this->get_logger(), "Gait generator started. Trapezoid: low_r=%.3f high_r=%.3f h=%.3f z=%.3f",
                trap_low_r_, trap_high_r_, trap_h_, trap_z_);

    rclcpp::Rate rate(50);  // 50 Hz для плавного движения
    int log_counter = 0;
    
    while (rclcpp::ok()) {
        // Only run gait if scale > 0 to avoid KDL path errors
        if (gait_command_.cmd == crab_msgs::msg::GaitCommand::RUNRIPPLE && gait_command_.scale > 0.01) {
            if (log_counter % 50 == 0) {
                RCLCPP_INFO(this->get_logger(), "RUNRIPPLE: fi=%.2f scale=%.2f alpha=%.2f", 
                            gait_command_.fi, gait_command_.scale, gait_command_.alpha);
            }
            final_vector = gait.RunRipple(frames_.begin(), gait_command_.fi, gait_command_.scale,
                                          gait_command_.alpha, d_ripple_);
            if (callService(final_vector)) {
                joints_pub_->publish(legs_);
                if (log_counter % 50 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Published joints: leg0=[%.3f,%.3f,%.3f]",
                                legs_.joints_state[0].joint[0], legs_.joints_state[0].joint[1], 
                                legs_.joints_state[0].joint[2]);
                }
            }
        } else if (gait_command_.cmd == crab_msgs::msg::GaitCommand::RUNTRIPOD && gait_command_.scale > 0.01) {
            if (log_counter % 50 == 0) {
                RCLCPP_INFO(this->get_logger(), "RUNTRIPOD: fi=%.2f scale=%.2f alpha=%.2f", 
                            gait_command_.fi, gait_command_.scale, gait_command_.alpha);
            }
            final_vector = gait.RunTripod(frames_.begin(), gait_command_.fi, gait_command_.scale,
                                          gait_command_.alpha, d_tripod_);
            if (callService(final_vector)) {
                joints_pub_->publish(legs_);
                if (log_counter % 50 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Published joints: leg0=[%.3f,%.3f,%.3f]",
                                legs_.joints_state[0].joint[0], legs_.joints_state[0].joint[1], 
                                legs_.joints_state[0].joint[2]);
                }
            }
        } else if (gait_command_.cmd == crab_msgs::msg::GaitCommand::PAUSE) {
            gait.Pause();
        } else {
            // STOP or invalid command or scale too small
            gait.Stop();
        }
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
        log_counter++;
    }
}

void GaitKinematics::teleopGaitCtrl(const crab_msgs::msg::GaitCommand::SharedPtr gait_cmd) {
    static int prev_cmd = -1;
    if (gait_cmd->cmd != prev_cmd) {
        RCLCPP_INFO(this->get_logger(), "Received gait command: cmd=%d fi=%.2f scale=%.2f alpha=%.2f",
                    gait_cmd->cmd, gait_cmd->fi, gait_cmd->scale, gait_cmd->alpha);
        prev_cmd = gait_cmd->cmd;
    }
    gait_command_.cmd = gait_cmd->cmd;
    gait_command_.fi = gait_cmd->fi;
    gait_command_.alpha = gait_cmd->alpha;
    gait_command_.scale = gait_cmd->scale;
}

bool GaitKinematics::callService(KDL::Vector* vector) {
    auto request = std::make_shared<crab_msgs::srv::GetLegIKSolver::Request>();

    for (size_t i = 0; i < num_legs_; i++) {
        request->leg_number.push_back(i);
        crab_msgs::msg::LegPositionState leg_pos_buf;
        leg_pos_buf.x = vector[i].x();
        leg_pos_buf.y = vector[i].y();
        leg_pos_buf.z = vector[i].z();
        request->target_position.push_back(leg_pos_buf);
        request->current_joints.push_back(legs_.joints_state[i]);
    }

    if (!client_->wait_for_service(100ms)) {
        return false;
    }

    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 100ms) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->error_codes == crab_msgs::srv::GetLegIKSolver::Response::IK_FOUND) {
            for (size_t i = 0; i < num_legs_; i++) {
                for (size_t j = 0; j < num_joints_; j++) {
                    legs_.joints_state[i].joint[j] = response->target_joints[i].joint[j];
                }
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "An IK solution could not be found");
            return false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        return false;
    }
    return true;
}

bool GaitKinematics::loadModel(const std::string& xml) {
    KDL::Tree tree;

    if (!kdl_parser::treeFromString(xml, tree)) {
        RCLCPP_ERROR(this->get_logger(), "Could not initialize tree object");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Construct tree");

    std::map<std::string, KDL::TreeElement>::const_iterator segments_iter;
    std::string link_name_result;
    for (size_t i = 0; i < num_legs_; i++) {
        link_name_result = root_name_ + suffixes[i];
        segments_iter = tree.getSegment(link_name_result);
        frames_.push_back((*segments_iter).second.segment.getFrameToTip());
    }
    for (size_t i = 0; i < num_legs_; i++) {
        link_name_result = tip_name_ + suffixes[i];
        segments_iter = tree.getSegment(link_name_result);
        frames_.push_back((*segments_iter).second.segment.getFrameToTip());
    }
    RCLCPP_INFO(this->get_logger(), "Get frames");

    for (size_t i = 0; i < num_legs_; i++) {
        frames_[i] = frames_[i] * frames_[i + num_legs_] * KDL::Frame(KDL::Vector(leg_radius_, 0, 0));
    }
    frames_.resize(num_legs_);

    for (size_t i = 0; i < num_legs_; i++) {
        for (size_t j = 0; j < num_joints_; j++) {
            legs_.joints_state[i].joint[j] = 0;
        }
    }

    return true;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GaitKinematics>();
    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "Could not initialize gait node");
        return -1;
    }
    node->gaitGenerator();
    rclcpp::shutdown();
    return 0;
}
