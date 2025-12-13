#include "gait.hpp"

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
    path_support_ = new KDL::Path_Line(d, a, &rot_, path_tolerance_, true);

    path_transfer_ = new KDL::Path_RoundedComposite(rounded_radius_, path_tolerance_, &rot_);
    path_transfer_->Add(a);
    path_transfer_->Add(b);
    path_transfer_->Add(c);
    path_transfer_->Add(d);
    path_transfer_->Finish();
}

void Gait::setTrajectory(double sup_path_duration, double tran_path_duration) {
    prof_support_.SetProfileDuration(0, path_support_->PathLength(), sup_path_duration);
    prof_transfer_.SetProfileDuration(0, path_transfer_->PathLength(), tran_path_duration);
    trajectory_transfer = new KDL::Trajectory_Segment(path_transfer_, &prof_transfer_);
    trajectory_support = new KDL::Trajectory_Segment(path_support_, &prof_support_);
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
    for (int i = phase_; i < num_legs_; i += 2) {
        frame = trajectory_transfer->Pos(passed_sec_);
        frame.p.x(frame.p.data[0] * scale);
        frame.p.y(frame.p.data[1] * scale);
        final_vector_[i] = frame.M * (*(vector_iter + i)).p + frame.p;
    }
    for (int i = !phase_; i < num_legs_; i += 2) {
        frame = trajectory_support->Pos(passed_sec_);
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

    getTipVector(trajectory_transfer, phase_offset, vector_iter, scale);
    getTipVector(trajectory_transfer, 0, vector_iter, scale);
    getTipVector(trajectory_support, 3 * phase_offset, vector_iter, scale);
    getTipVector(trajectory_support, 2 * phase_offset, vector_iter, scale);
    getTipVector(trajectory_support, phase_offset, vector_iter, scale);
    getTipVector(trajectory_support, 0, vector_iter, scale);

    if (passed_sec_ >= phase_offset - 0.02 && run_state_ == true) {
        begin_sec_ = now_sec;
        passed_sec_ = 0;

        legs_queue_.push(legs_queue_.front());
        legs_queue_.pop();
    }
    return final_vector_;
}

void Gait::getTipVector(KDL::Trajectory_Segment *trajectory, double phase_offset, std::vector<KDL::Frame>::const_iterator vector_iter, double scale) {
    KDL::Frame frame;
    frame = trajectory->Pos(passed_sec_ + phase_offset);
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
