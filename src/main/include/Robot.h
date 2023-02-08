// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <vector>

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/RelativeEncoder.h>
#include "AutoPath.h"

#define lMotorLeaderID 1
#define lMotorFollowerID 2
#define rMotorLeaderID 4
#define rMotorFollowerID 3

#define PIDProportional 0.59
#define PIDIntegral 0
#define PIDDerivative 0.28
#define PIDIZone 0


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  frc::XboxController* ctr = new frc::XboxController(0);

  AutoPath pathModule;

  rev::CANSparkMax* lMotor = new rev::CANSparkMax(lMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* lMotorFollower = new rev::CANSparkMax(lMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* rMotor = new rev::CANSparkMax(rMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* rMotorFollower = new rev::CANSparkMax(rMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);

  rev::SparkMaxRelativeEncoder lEncoder = lMotor->GetEncoder();
  rev::SparkMaxRelativeEncoder rEncoder = rMotor->GetEncoder();
  rev::SparkMaxPIDController lPID = lMotor->GetPIDController();
  rev::SparkMaxPIDController rPID = rMotor->GetPIDController();
 private:

};
