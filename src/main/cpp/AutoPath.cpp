#include "AutoPath.h"


void AutoPath::Init() {
    timer.Start();
    getPaths();
}

std::vector<double> AutoPath::Periodic(double angle, double left_enc, double right_enc) {
    m_odometry.Update(frc::Rotation2d(units::degree_t{angle}), units::meter_t{left_enc}, units::meter_t{right_enc});
    frc::Trajectory::State goal = trajectory.Sample(timer.Get());
    frc::ChassisSpeeds speeds = r_ctr.Calculate(m_odometry.GetPose(), goal);
    auto [left, right] = drive_kinematics.ToWheelSpeeds(speeds);
    double left_out = units::feet_per_second_t{left}.value(); // meters per sec
    double right_out = units::feet_per_second_t{right}.value(); // meters per sec
    return std::vector{left_out, right_out};

}

void AutoPath::getPaths() {
   fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
   deployDirectory = deployDirectory / "pathplanner" / "generatedJSON" / "Example.wpilib.json";
   trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
} 