// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "gyro.h"
#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

gyro Gyro;

void Robot::RobotInit() {
  Gyro = gyro();
  lMotor->SetInverted(false);
  rMotor->SetInverted(true);

  lEncoder.SetPositionConversionFactor((180 / M_PI) * 64 / 12);
  rEncoder.SetPositionConversionFactor((180 / M_PI) * 64 / 12);

  lMotorFollower->Follow(*lMotor, false);
  rMotorFollower->Follow(*rMotor, false);

  lMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  lMotorFollower->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  rMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  rMotorFollower->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  lEncoder.SetPosition(0);
  rEncoder.SetPosition(0);

  lPID.SetP(PIDProportional);
  lPID.SetI(PIDIntegral);
  lPID.SetD(PIDDerivative);
  lPID.SetIZone(PIDIZone);

  rPID.SetP(PIDProportional);
  rPID.SetI(PIDIntegral);
  rPID.SetD(PIDDerivative);
  rPID.SetIZone(PIDIZone);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  pathModule.Init();


}

void Robot::AutonomousPeriodic() {
  std::vector<double> wheel_speeds = pathModule.Periodic(Gyro.ahrs->GetAngle(), lEncoder.GetPosition(), rEncoder.GetPosition());
  lPID.SetReference(wheel_speeds.at(0), rev::CANSparkMax::ControlType::kVelocity);
  rPID.SetReference(wheel_speeds.at(1), rev::CANSparkMax::ControlType::kVelocity);
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  double lefty =ctr->GetLeftY();
  double righty = ctr->GetRightY();
  frc::SmartDashboard::PutNumber("CTR_LEFTY", lefty);
  frc::SmartDashboard::PutNumber("CTR_RIGHTY", righty);
  lMotor->Set(lefty / 2);
  rMotor->Set(righty / 2);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
