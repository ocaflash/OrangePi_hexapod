#include "gait.hpp"
#include <algorithm>
#include <cmath>
#include <kdl/velocityprofile_spline.hpp>

namespace {
double clamp_time(const KDL::Trajectory_Segment* trajectory, double t) {
    if (!trajectory) {
        return t;
    }
    const double duration = trajectory->Duration();
    if (duration <= 0.0) {
        return 0.0;
    }
    return std::min(std::max(0.0, t), duration);
}
}  // namespace

Gait::Gait() : run_state_(false), pause_state_(false), phase_(0), passed_sec_(0), begin_sec_(0), cycle_time_(0) {
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
    // KDL trajectory/path classes are known to manage (delete) the objects passed in.
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
    // Tripod gait: 3 ноги в воздухе, 3 на земле
    // Группа A: R1(0), L2(4), R3(2)
    // Группа B: L1(3), R2(1), L3(5)
    
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
    
    // Группы ног для tripod gait
    static const int group_a[3] = {0, 4, 2};  // R1, L2, R3
    static const int group_b[3] = {3, 1, 5};  // L1, R2, L3
    
    // Группа в фазе переноса (transfer)
    const int* transfer_group = (phase_ == 0) ? group_a : group_b;
    const int* support_group = (phase_ == 0) ? group_b : group_a;
    
    for (int i = 0; i < 3; i++) {
        int leg_idx = transfer_group[i];
        frame = trajectory_transfer_->Pos(clamp_time(trajectory_transfer_.get(), passed_sec_));
        frame.p.x(frame.p.data[0] * scale);
        frame.p.y(frame.p.data[1] * scale);
        final_vector_[leg_idx] = frame.M * (*(vector_iter + leg_idx)).p + frame.p;
    }
    
    for (int i = 0; i < 3; i++) {
        int leg_idx = support_group[i];
        frame = trajectory_support_->Pos(clamp_time(trajectory_support_.get(), passed_sec_));
        frame.p.x(frame.p.data[0] * scale);
        frame.p.y(frame.p.data[1] * scale);
        final_vector_[leg_idx] = frame.M * (*(vector_iter + leg_idx)).p + frame.p;
    }

    if (passed_sec_ >= duration - 0.02 && run_state_ == true) {
        begin_sec_ = now_sec;
        passed_sec_ = 0;
        phase_ = !phase_;
    }
    return final_vector_;
}

KDL::Vector* Gait::RunRipple(std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration) {
    // Ripple gait: 2 ноги в воздухе, 4 на земле в любой момент времени
    // Период одной ноги = duration
    // Фаза переноса (transfer) = 1/3 периода
    // Фаза опоры (support) = 2/3 периода
    // Смещение между ногами = 1/6 периода
    
    double phase_offset = duration / num_legs_;
    double transfer_duration = duration / 3.0;
    double support_duration = duration * 2.0 / 3.0;
    
    setFi(fi);
    setAlpha(alpha);
    setPath();
    setTrajectory(support_duration, transfer_duration);

    auto now_sec = rclcpp::Clock().now().seconds();

    if (run_state_ == false) {
        run_state_ = true;
        begin_sec_ = now_sec;
        passed_sec_ = 0;
        cycle_time_ = 0;
    }

    if (pause_state_ == true) {
        begin_sec_ = now_sec - passed_sec_;
        pause_state_ = false;
    }

    passed_sec_ = now_sec - begin_sec_;
    
    // Общее время цикла (непрерывно растёт)
    cycle_time_ = passed_sec_;
    
    // Порядок ног для ripple gait: противоположные ноги с разных сторон
    // R1(0), L3(5), R3(2), L1(3), R2(1), L2(4) - классический ripple
    static const int leg_order[6] = {0, 5, 2, 3, 1, 4};
    
    for (int i = 0; i < num_legs_; i++) {
        int leg_idx = leg_order[i];
        
        // Фаза для этой ноги (с учётом смещения)
        double leg_phase = fmod(cycle_time_ + i * phase_offset, duration);
        
        KDL::Frame frame;
        
        // Определяем, в какой фазе находится нога
        if (leg_phase < transfer_duration) {
            // Фаза переноса (нога в воздухе)
            double t = leg_phase;
            frame = trajectory_transfer_->Pos(clamp_time(trajectory_transfer_.get(), t));
        } else {
            // Фаза опоры (нога на земле)
            double t = leg_phase - transfer_duration;
            frame = trajectory_support_->Pos(clamp_time(trajectory_support_.get(), t));
        }
        
        frame.p.x(frame.p.data[0] * scale);
        frame.p.y(frame.p.data[1] * scale);
        final_vector_[leg_idx] = frame.M * (*(vector_iter + leg_idx)).p + frame.p;
    }

    return final_vector_;
}

void Gait::getTipVector(const KDL::Trajectory_Segment* trajectory, double phase_offset,
                        std::vector<KDL::Frame>::const_iterator vector_iter, double scale) {
    KDL::Frame frame;
    frame = trajectory->Pos(clamp_time(trajectory, passed_sec_ + phase_offset));
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
