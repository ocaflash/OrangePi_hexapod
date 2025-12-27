#include "gait.hpp"
#include <algorithm>
#include <cmath>
#include <kdl/velocityprofile_spline.hpp>

Gait::Gait() : run_state_(false), pause_state_(false), phase_(0), passed_sec_(0), begin_sec_(0) {
    legs_queue_.push(5);
    legs_queue_.push(0);
    legs_queue_.push(4);
    legs_queue_.push(2);
    legs_queue_.push(3);
    legs_queue_.push(1);
}

void Gait::setTrapezoid(double low_r, double high_r, double h, double z,
                        double path_tolerance, double rounded_radius) {
    low_rad_ = low_r;
    high_rad_ = high_r;
    height_ = z - h;
    z_body_ = z;
    path_tolerance_ = path_tolerance;
    rounded_radius_ = rounded_radius;
}

void Gait::setFi(double fi) {
    a.p = KDL::Vector(-low_rad_ * cos(fi), -low_rad_ * sin(fi), -z_body_);
    b.p = KDL::Vector(-high_rad_ * cos(fi), -high_rad_ * sin(fi), -height_);
    c.p = KDL::Vector(high_rad_ * cos(fi), high_rad_ * sin(fi), -height_);
    d.p = KDL::Vector(low_rad_ * cos(fi), low_rad_ * sin(fi), -z_body_);
}

void Gait::setAlpha(double alpha) {
    a.M = KDL::Rotation::RotZ(-alpha);
    b.M = KDL::Rotation::RotZ(-alpha / 2);
    c.M = KDL::Rotation::RotZ(alpha / 2);
    d.M = KDL::Rotation::RotZ(alpha);
}

void Gait::setPath() {
    // Paths are created inside setTrajectory() so that Trajectory_Segment can own all KDL
    // heap objects consistently (avoids invalid free with stack/member pointers).
}

void Gait::setTrajectory(double sup_path_duration, double tran_path_duration) {
    // Destroy old trajectories first (they may free their internal objects)
    trajectory_support_.reset();
    trajectory_transfer_.reset();

    // Allocate everything on heap and hand pointers to Trajectory_Segment.
    auto* rot_support = new KDL::RotationalInterpolation_SingleAxis();
    auto* rot_transfer = new KDL::RotationalInterpolation_SingleAxis();

    auto* path_support = new KDL::Path_Line(d, a, rot_support, path_tolerance_, true);
    auto* path_transfer = new KDL::Path_RoundedComposite(rounded_radius_, path_tolerance_, rot_transfer);
    path_transfer->Add(a);
    path_transfer->Add(b);
    path_transfer->Add(c);
    path_transfer->Add(d);
    path_transfer->Finish();

    auto* prof_support = new KDL::VelocityProfile_Spline();
    auto* prof_transfer = new KDL::VelocityProfile_Spline();
    prof_support->SetProfileDuration(0, path_support->PathLength(), sup_path_duration);
    prof_transfer->SetProfileDuration(0, path_transfer->PathLength(), tran_path_duration);

    trajectory_support_ = std::make_unique<KDL::Trajectory_Segment>(path_support, prof_support);
    trajectory_transfer_ = std::make_unique<KDL::Trajectory_Segment>(path_transfer, prof_transfer);
}

KDL::Vector* Gait::RunTripod(std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration) {
    setFi(fi);
    setAlpha(alpha);
    setPath();
    setTrajectory(duration, duration);
    KDL::Frame frame;

    auto now_sec = rclcpp::Clock().now().seconds();

    if (run_state_ == false) {
        run_state_ = true;
        phase_ = 0;
        begin_sec_ = now_sec;
        passed_sec_ = 0;
    }
    if (pause_state_ == true) {
        begin_sec_ = now_sec - passed_sec_;
        pause_state_ = false;
    }

    passed_sec_ = now_sec - begin_sec_;
    
    // Clamp passed_sec_ to trajectory duration
    double t_transfer = std::min(std::max(0.0, passed_sec_), trajectory_transfer_->Duration());
    double t_support = std::min(std::max(0.0, passed_sec_), trajectory_support_->Duration());
    
    for (int i = phase_; i < num_legs_; i += 2) {
        frame = trajectory_transfer_->Pos(t_transfer);
        frame.p.x(frame.p.data[0] * scale);
        frame.p.y(frame.p.data[1] * scale);
        final_vector_[i] = frame.M * (*(vector_iter + i)).p + frame.p;
    }
    for (int i = !phase_; i < num_legs_; i += 2) {
        frame = trajectory_support_->Pos(t_support);
        frame.p.x(frame.p.data[0] * scale);
        frame.p.y(frame.p.data[1] * scale);
        final_vector_[i] = frame.M * (*(vector_iter + i)).p + frame.p;
    }

    if (passed_sec_ >= duration - 0.02 && run_state_ == true) {
        begin_sec_ = now_sec;
        passed_sec_ = 0;
        phase_ = !phase_;
    }
    return final_vector_;
}

KDL::Vector* Gait::RunRipple(std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration) {
    double phase_offset = duration / num_legs_;
    setFi(fi);
    setAlpha(alpha);
    setPath();
    setTrajectory(duration * 2 / 3, duration / 3);

    auto now_sec = rclcpp::Clock().now().seconds();

    if (run_state_ == false) {
        run_state_ = true;
        begin_sec_ = now_sec;
        passed_sec_ = 0;
    }

    if (pause_state_ == true) {
        begin_sec_ = now_sec - passed_sec_;
        pause_state_ = false;
    }

    passed_sec_ = now_sec - begin_sec_;

    getTipVector(trajectory_transfer_.get(), phase_offset, vector_iter, scale);
    getTipVector(trajectory_transfer_.get(), 0, vector_iter, scale);
    getTipVector(trajectory_support_.get(), 3 * phase_offset, vector_iter, scale);
    getTipVector(trajectory_support_.get(), 2 * phase_offset, vector_iter, scale);
    getTipVector(trajectory_support_.get(), phase_offset, vector_iter, scale);
    getTipVector(trajectory_support_.get(), 0, vector_iter, scale);

    if (passed_sec_ >= phase_offset - 0.02 && run_state_ == true) {
        begin_sec_ = now_sec;
        passed_sec_ = 0;

        legs_queue_.push(legs_queue_.front());
        legs_queue_.pop();
    }
    return final_vector_;
}

void Gait::getTipVector(const KDL::Trajectory_Segment* trajectory, double phase_offset,
                        std::vector<KDL::Frame>::const_iterator vector_iter, double scale) {
    KDL::Frame frame;
    double t = passed_sec_ + phase_offset;
    // Clamp time to trajectory duration
    t = std::min(std::max(0.0, t), trajectory->Duration());
    frame = trajectory->Pos(t);
    frame.p.x(frame.p.data[0] * scale);
    frame.p.y(frame.p.data[1] * scale);
    final_vector_[legs_queue_.front()] = frame.M * (*(vector_iter + legs_queue_.front())).p + frame.p;
    legs_queue_.push(legs_queue_.front());
    legs_queue_.pop();
}

void Gait::Pause() {
    pause_state_ = true;
}

void Gait::Stop() {
    run_state_ = false;
}
