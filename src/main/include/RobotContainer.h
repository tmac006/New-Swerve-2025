// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/DrivebaseSubsystem.h"

#include <functional>

using DeadbandAndSquareFunc = std::function<double()>;

class RobotContainer {
 public:
  RobotContainer();

  DrivebaseSubsystem& GetDrivebaseSubsystem();

  frc2::CommandPtr GetAutonomousCommand();

  units::radian_t desiredAngle = 0_rad;

 private:
  
  void ConfigureBindings();
  frc2::CommandXboxController m_driverController{0};
  //frc2::CommandXboxController m_topController{1};

  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;
   DrivebaseSubsystem driveSub;

    frc2::CommandPtr selfTestCmd = driveSub.SelfTest();
  frc2::CommandPtr measureWheelCmd = driveSub.MeasureWheelDiam([] {
    return frc::SmartDashboard::GetBoolean("Drivebase/DoneWithStep", false);
  });
  frc2::CommandPtr tuneSteerCmd = driveSub.TuneSteerPID([] {
    return frc::SmartDashboard::GetBoolean("Drivebase/DoneWithStep", false);
  });
  frc2::CommandPtr tuneDriveCmd = driveSub.TuneDrivePID([] {
    return frc::SmartDashboard::GetBoolean("Drivebase/DoneWithStep", false);
  });
  
  double rotSpeed = 0;

    frc2::CommandPtr resetPositionCmd =
      driveSub.ResetPosition([] { return frc::Pose2d{}; });
  frc2::CommandPtr tunePathPidCmd = driveSub.TunePathPid();
  frc2::CommandPtr donePathTuningCmd = driveSub.DoneTuningPathPids();
  frc2::CommandPtr zeroYawCMD = driveSub.ZeroYawCMD();

  std::function<frc2::CommandPtr()> GetAStarCmd();

  DeadbandAndSquareFunc DeadbandAndSquare(
      std::function<double()> joystickValue);

  double ShouldFlipControlsForDriver(double val);
  units::radian_t ShouldFlipAngleForDriver(units::radian_t targetAngle);
};
