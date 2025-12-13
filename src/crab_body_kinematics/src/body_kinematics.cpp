#include "body_kinematics.hpp"

using namespace std::chrono_literals;

const static std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};

BodyKinematics::BodyKinematics() : Node("body_kinematics") {
    // Declare parameters
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("root_name_body", "leg_center");
    this->declare_parameter<std::string>("tip_name_body", "coxa");
    this->declare_parameter<double>("clearance", 0.045);
}

bool BodyKinematics::init() {
    std::string robot_desc_string;
    
    // Get URDF XML
    robot_desc_string = this->get_parameter("robot_description").as_string();
    if (robot_desc_string.empty()) {
        RCLCPP_FATAL(this->get_logger(), "Could not load the xml from parameter: robot_description");
        return false;
    }

    // Get Root and Tip From Parameter Server
    root_name_ = this->get_parameter("root_name_body").as_string();
    tip_name_ = this->get_parameter("tip_name_body").as_string();
    z_ = this->get_parameter("clearance").as_double();

    // Load and Read Models
    if (!loadModel(robot_desc_string)) {
        RCLCPP_FATAL(this->get_logger(), "Could not load models!");
        return false;
    }

    client_ = this->create_client<crab_msgs::srv::GetLegIKSolver>("/crab_leg_kinematics/get_ik");
    joints_pub_ = this->create_publisher<crab_msgs::msg::LegsJointsState>("joints_to_controller", 1);
    body_move_sub_ = this->create_subscription<crab_msgs::msg::BodyState>(
        "/teleop/move_body", 1,
        std::bind(&BodyKinematics::teleopBodyMove, this, std::placeholders::_1));
    body_cmd_sub_ = this->create_subscription<crab_msgs::msg::BodyCommand>(
        "/teleop/body_command", 1,
        std::bind(&BodyKinematics::teleopBodyCmd, this, std::placeholders::_1));
    
    // Timer for smooth stand/seat movements
    motion_timer_ = this->create_wall_timer(
        40ms, std::bind(&BodyKinematics::motionTimerCallback, this));

    bs_.leg_radius = 0.11;
    bs_.z = -0.016;
    stand_up_active_ = false;
    seat_down_active_ = false;
    
    // Ждём пока сервис IK станет доступен
    RCLCPP_INFO(this->get_logger(), "Waiting for leg IK service...");
    while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Still waiting for leg IK service...");
    }
    RCLCPP_INFO(this->get_logger(), "Leg IK service available");
    
    calculateKinematics(&bs_);
    RCLCPP_INFO(this->get_logger(), "Ready to receive teleop messages...");

    return true;
}

bool BodyKinematics::loadModel(const std::string& xml) {
    // Construct tree with kdl_parser
    KDL::Tree tree;

    if (!kdl_parser::treeFromString(xml, tree)) {
        RCLCPP_ERROR(this->get_logger(), "Could not initialize tree object");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Construct tree");

    // Get coxa and leg_center frames via segments
    std::map<std::string, KDL::TreeElement>::const_iterator segments_iter;
    std::string link_name_result;
    for (int i = 0; i < num_legs_; i++) {
        link_name_result = root_name_ + suffixes[i];
        segments_iter = tree.getSegment(link_name_result);
        frames_.push_back((*segments_iter).second.segment.getFrameToTip());
    }
    for (int i = 0; i < num_legs_; i++) {
        link_name_result = tip_name_ + suffixes[i];
        segments_iter = tree.getSegment(link_name_result);
        frames_.push_back((*segments_iter).second.segment.getFrameToTip());
    }
    RCLCPP_INFO(this->get_logger(), "Get frames");

    // Vector iterators
    for (size_t i = 0; i < num_legs_; i++) {
        frames_[i] = frames_[i] * frames_[i + num_legs_];
    }
    frames_.resize(num_legs_);

    for (size_t i = 0; i < num_legs_; i++) {
        for (size_t j = 0; j < num_joints_; j++) {
            legs_.joints_state[i].joint[j] = 0;
        }
    }

    return true;
}

bool BodyKinematics::calculateKinematics(crab_msgs::msg::BodyState* body_ptr) {
    // Body rotation
    rotation_ = KDL::Rotation::RPY(body_ptr->roll, body_ptr->pitch, body_ptr->yaw);

    // Distance from body center to leg tip
    femur_frame_ = KDL::Frame(KDL::Vector(body_ptr->leg_radius, 0, 0));

    // Offset from center
    offset_vector_ = KDL::Vector(body_ptr->x, body_ptr->y, body_ptr->z);
    rotate_correction_ = KDL::Vector(
        body_ptr->z * tan(body_ptr->pitch),
        -(body_ptr->z * tan(body_ptr->roll)),
        0);

    for (size_t i = 0; i < num_legs_; i++) {
        // Get tip frames
        tibia_foot_frame_ = frames_[i] * femur_frame_;
        // Get tip vectors with body position
        final_vector_[i] = (rotation_ * tibia_foot_frame_.p) + offset_vector_ + rotate_correction_;
    }

    if (!callService(final_vector_)) {
        return false;
    }

    return true;
}

bool BodyKinematics::callService(KDL::Vector* vector) {
    auto request = std::make_shared<crab_msgs::srv::GetLegIKSolver::Request>();
    
    // Creating message to request
    for (size_t i = 0; i < num_legs_; i++) {
        request->leg_number.push_back(i);
        crab_msgs::msg::LegPositionState leg_pos_buf;
        leg_pos_buf.x = vector[i].x();
        leg_pos_buf.y = vector[i].y();
        leg_pos_buf.z = vector[i].z();
        request->target_position.push_back(leg_pos_buf);
        request->current_joints.push_back(legs_.joints_state[i]);
    }

    // Check service availability (non-blocking)
    if (!client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Service not available");
        return false;
    }

    // Send async request with callback
    auto future = client_->async_send_request(request,
        [this](rclcpp::Client<crab_msgs::srv::GetLegIKSolver>::SharedFuture response_future) {
            auto response = response_future.get();
            if (response->error_codes == crab_msgs::srv::GetLegIKSolver::Response::IK_FOUND) {
                for (size_t i = 0; i < num_legs_; i++) {
                    for (size_t j = 0; j < num_joints_; j++) {
                        legs_.joints_state[i].joint[j] = response->target_joints[i].joint[j];
                    }
                }
                joints_pub_->publish(legs_);
            } else {
                RCLCPP_ERROR(this->get_logger(), "An IK solution could not be found");
            }
        });

    return true;
}

void BodyKinematics::teleopBodyMove(const crab_msgs::msg::BodyState::SharedPtr body_state) {
    bs_.x = body_state->x;
    bs_.y = body_state->y;
    bs_.z = body_state->z;
    bs_.pitch = body_state->pitch;
    bs_.roll = body_state->roll;
    bs_.yaw = body_state->yaw;
    bs_.leg_radius = body_state->leg_radius;
    calculateKinematics(&bs_);
}

void BodyKinematics::teleopBodyCmd(const crab_msgs::msg::BodyCommand::SharedPtr body_cmd) {
    if (body_cmd->cmd == crab_msgs::msg::BodyCommand::STAND_UP_CMD) {
        RCLCPP_INFO(this->get_logger(), "STAND_UP_CMD");
        // Start stand up sequence using timer
        stand_up_active_ = true;
        seat_down_active_ = false;
    }
    if (body_cmd->cmd == crab_msgs::msg::BodyCommand::SEAT_DOWN_CMD) {
        RCLCPP_INFO(this->get_logger(), "SEAT_DOWN_CMD");
        // Start seat down sequence using timer
        seat_down_active_ = true;
        stand_up_active_ = false;
    }
}

void BodyKinematics::motionTimerCallback() {
    if (stand_up_active_) {
        if (bs_.z >= -z_) {
            bs_.z -= 0.0025;
            RCLCPP_INFO(this->get_logger(), "Stand up: z=%.4f, target=-%.4f, leg0: x=%.4f y=%.4f z=%.4f",
                        bs_.z, z_, final_vector_[0].x(), final_vector_[0].y(), final_vector_[0].z());
            calculateKinematics(&bs_);
        } else {
            stand_up_active_ = false;
            RCLCPP_INFO(this->get_logger(), "Stand up complete");
        }
    }
    if (seat_down_active_) {
        if (bs_.z <= -0.016) {
            bs_.z += 0.0025;
            calculateKinematics(&bs_);
        } else {
            seat_down_active_ = false;
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BodyKinematics>();
    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "Could not initialize kinematics node");
        return -1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
