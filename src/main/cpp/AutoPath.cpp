#include "AutoPath.h"


void AutoPath::Init() {
    timer.Start();
    getPaths();
}

std::vector<double> AutoPath::Periodic(double angle, double left_enc, double right_enc) {
    m_odometry.Update(frc::Rotation2d(units::degree_t{angle}), units::foot_t{left_enc}, units::foot_t{right_enc});
    frc::Trajectory::State goal = examplePath.sample(timer.Get()).asWPILibState();
    frc::ChassisSpeeds speeds = r_ctr.Calculate(m_odometry.GetPose(), goal);
    auto [left, right] = drive_kinematics.ToWheelSpeeds(speeds);
    double left_out = units::feet_per_second_t{left}.value(); // meters per sec
    double right_out = units::feet_per_second_t{right}.value(); // meters per sec
    return std::vector{left_out, right_out};

}

void AutoPath::getPaths() {
    pathplanner::PathPlannerTrajectory examplePath = pathplanner::PathPlanner::loadPath("Example", pathplanner::PathConstraints(3.49_mps, 6.71_mps_sq));
    //PathPlannerTrajectory::PathPlannerState exampleState = examplePath.sample(1.2_s);
    //exampleState
    /*
   fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
   deployDirectory = deployDirectory / "pathplanner" / "generatedJSON" / "Example.wpilib.json";
   trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
   //trajectory = frc::Traje
   */
} 