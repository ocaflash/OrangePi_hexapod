#ifndef GAIT_HPP_
#define GAIT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <kdl/frames.hpp>
#include <kdl/path_line.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <queue>
#include <memory>

#define NUM_LEGS 6
#define NUM_JOINTS 3

class Gait {
public:
    Gait();
    void setTrapezoid(double low_rad, double high_rad, double height, double z,
                      double path_tolerance = 0.005, double rounded_radius = 0.02);
    KDL::Vector* RunTripod(std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration);
    KDL::Vector* RunRipple(std::vector<KDL::Frame>::const_iterator vector_iter, double fi, double scale, double alpha, double duration);
    void Pause();
    void Stop();

    KDL::Frame a, b, c, d;

private:
    static constexpr unsigned int num_joints_ = NUM_JOINTS;
    static constexpr unsigned int num_legs_ = NUM_LEGS;

    void getTipVector(const KDL::Trajectory_Segment* trajectory, double phase_offset,
                      std::vector<KDL::Frame>::const_iterator vector_iter, double scale);
    void setFi(double fi);
    void setAlpha(double alpha);
    void setPath();
    void setTrajectory(double sup_path_duration, double tran_path_duration);

    std::queue<int> legs_queue_;
    bool run_state_, pause_state_;
    int phase_;
    double passed_sec_, begin_sec_;
    double low_rad_, high_rad_, height_, z_body_;
    double path_tolerance_, rounded_radius_;
    KDL::Vector final_vector_[NUM_LEGS];
    // NOTE: Trajectories reference paths, so they must be destroyed before the paths.
    std::unique_ptr<KDL::Trajectory_Segment> trajectory_transfer_;
    std::unique_ptr<KDL::Trajectory_Segment> trajectory_support_;
};

#endif /* GAIT_HPP_ */
