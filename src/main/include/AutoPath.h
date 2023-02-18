
#include<frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/controller/RamseteController.h>
#include<frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc/geometry/Rotation2d.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/Timer.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathConstraints.h>



#include <vector>



class AutoPath {
    public:
    double b = 2.0;
    double zeta = 0.7;
    double track_width = 29 / 12; //ft
    //w is change in gyro use Getrate
    frc::Timer timer;

    void Init();
    std::vector<double> Periodic(double angle, double left_enc, double right_enc);
    void getPaths();
    

    frc::DifferentialDriveOdometry m_odometry {frc::Rotation2d(), units::meter_t{0}, units::meter_t{0}};

    frc::RamseteController r_ctr;

    //frc::Trajectory trajectory;
    pathplanner::PathPlannerTrajectory examplePath;

    frc::DifferentialDriveKinematics drive_kinematics{units::foot_t(track_width)};
    private:
    

};