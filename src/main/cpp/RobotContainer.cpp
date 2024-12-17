// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>
#include "commands/Autos.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() 
{
   driveSub.SetDefaultCommand(driveSub.DriveFactory(
      DeadbandAndSquare([this] { return -m_driverController.GetLeftY(); }),
      DeadbandAndSquare([this] { return -m_driverController.GetLeftX(); }),
      DeadbandAndSquare([this] { return -m_driverController.GetRightX(); }),
      [] { return true; }));

  // m_driverController.Y().OnTrue(driveSub.TurnToAngleFactory(
  //     DeadbandAndSquare([this] { return -m_driverController.GetLeftY(); }),
  //     DeadbandAndSquare([this] { return -m_driverController.GetLeftX(); }),
  //     [this] {
  //       return frc::TrapezoidProfile<units::radians>::State{
  //           ShouldFlipAngleForDriver(180_deg), 0_deg_per_s};
  //     },
  //     [this] { return std::abs(m_driverController.GetRightX()) > 0.1; }, true));

  // m_driverController.X().OnTrue(driveSub.TurnToAngleFactory(
  //     DeadbandAndSquare([this] { return -m_driverController.GetLeftY(); }),
  //     DeadbandAndSquare([this] { return -m_driverController.GetLeftX(); }),
  //     [this] {
  //       return frc::TrapezoidProfile<units::radians>::State{
  //           ShouldFlipAngleForDriver(-90_deg), 0_deg_per_s};
  //     },
  //     [this] { return std::abs(m_driverController.GetRightX()) > 0.1; }, true));

  // m_driverController.B().OnTrue(driveSub.TurnToAngleFactory(
  //     DeadbandAndSquare([this] { return -m_driverController.GetLeftY(); }),
  //     DeadbandAndSquare([this] { return -m_driverController.GetLeftX(); }),
  //     [this] {
  //       return frc::TrapezoidProfile<units::radians>::State{
  //           ShouldFlipAngleForDriver(90_deg), 0_deg_per_s};
  //     },
  //     [this] { return std::abs(m_driverController.GetRightX()) > 0.1; }, true));

  // m_driverController.A().OnTrue(driveSub.TurnToAngleFactory(
  //     DeadbandAndSquare([this] { return -m_driverController.GetLeftY(); }),
  //     DeadbandAndSquare([this] { return -m_driverController.GetLeftX(); }),
  //     [this] {
  //       return frc::TrapezoidProfile<units::radians>::State{
  //           ShouldFlipAngleForDriver(0_deg), 0_deg_per_s};
  //     },
  //     [this] { return std::abs(m_driverController.GetRightX()) > 0.1; }, true));

  // driverController.RightBumper().OnTrue(driveSub.TurnToAngleFactory(
  //     DeadbandAndSquare([this] { return -driverController.GetLeftY(); }),
  //     DeadbandAndSquare([this] { return -driverController.GetLeftX(); }),
  //     [this] {
  //       frc::Translation2d goal =
  //           constants::swerve::automation::BLUE_ALLIANCE_GOAL;
  //       auto ally = frc::DriverStation::GetAlliance();
  //       if (ally.has_value()) {
  //         if (ally.value() == frc::DriverStation::Alliance::kRed) {
  //           goal = constants::swerve::automation::RED_ALLIANCE_GOAL;
  //         }
  //       }
  //       frc::Pose2d pose = driveSub.GetRobotPose();
  //       frc::Rotation2d angle{
  //           units::math::atan2(goal.Y() - pose.Translation().Y(),
  //                              goal.X() - pose.Translation().X())};
  //       return frc::TrapezoidProfile<units::radians>::State{angle.Radians(),
  //                                                           0_deg_per_s};
  //     },
  //     [this] { return std::abs(driverController.GetRightX()) > 0.1; }, true));

  
  frc::SmartDashboard::PutBoolean("Drivebase/DoneWithStep", false);

  frc::SmartDashboard::PutData("Drivebase/SelfTestCmd", selfTestCmd.get());

  frc::SmartDashboard::PutData("Drivebase/MeasureWheelCmd",
                               measureWheelCmd.get());

  frc::SmartDashboard::PutData("Drivebase/TuneSteerCmd", tuneSteerCmd.get());

  frc::SmartDashboard::PutData("Drivebase/ZeroYawCMD", zeroYawCMD.get());

  frc::SmartDashboard::PutData("Drivebase/TuneDriveCmd", tuneDriveCmd.get());

  frc::SmartDashboard::PutData("Drivebase/ResetPosition",
                               resetPositionCmd.get());

  frc::SmartDashboard::PutData("Drivebase/PathTuningCmd", tunePathPidCmd.get());

  frc::SmartDashboard::PutData("Drivebase/DonePathTuningCmd",
                               donePathTuningCmd.get());

  // testController.Back().WhileTrue(driveSub.WheelRadFwd());
  // testController.Start().WhileTrue(driveSub.WheelRadRev());

  // testController.Back().WhileTrue(
  //     driveSub.SysIdQuasistaticSteer(frc2::sysid::Direction::kForward));
  // testController.Start().WhileTrue(
  //     driveSub.SysIdQuasistaticSteer(frc2::sysid::Direction::kReverse));
  // testController.LeftBumper().WhileTrue(
  //     driveSub.SysIdDynamicSteer(frc2::sysid::Direction::kForward));
  // testController.RightBumper().WhileTrue(
  //     driveSub.SysIdDynamicSteer(frc2::sysid::Direction::kReverse));

  // testController.A().WhileTrue(
  //     driveSub.SysIdQuasistaticDrive(frc2::sysid::Direction::kForward));
  // testController.B().WhileTrue(
  //     driveSub.SysIdQuasistaticDrive(frc2::sysid::Direction::kReverse));
  // testController.X().WhileTrue(
  //     driveSub.SysIdDynamicDrive(frc2::sysid::Direction::kForward));
  // testController.Y().WhileTrue(
  //     driveSub.SysIdDynamicDrive(frc2::sysid::Direction::kReverse));

  // testController.A().WhileTrue(
  //     shooterSub.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // testController.B().WhileTrue(
  //     shooterSub.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  // testController.X().WhileTrue(
  //     shooterSub.SysIdDynamic(frc2::sysid::Direction::kForward));
  // testController.Y().WhileTrue(
  //     shooterSub.SysIdDynamic(frc2::sysid::Direction::kReverse));

  // testController.A().WhileTrue(
  //     dunkSub.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // testController.B().WhileTrue(
  //     dunkSub.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  // testController.X().WhileTrue(
  //     dunkSub.SysIdDynamic(frc2::sysid::Direction::kForward));
  // testController.Y().WhileTrue(
  //     dunkSub.SysIdDynamic(frc2::sysid::Direction::kReverse));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() 
{
  return autos::ExampleAuto(&m_subsystem);
}

double RobotContainer::ShouldFlipControlsForDriver(double val) {
  auto ally = frc::DriverStation::GetAlliance();
  if (ally.has_value()) {
    if (ally.value() == frc::DriverStation::Alliance::kRed) {
      return val;
    }
  }
  return -val;
}

DrivebaseSubsystem& RobotContainer::GetDrivebaseSubsystem() 
{
  return driveSub;
}

DeadbandAndSquareFunc RobotContainer::DeadbandAndSquare(
    std::function<double()> joystickValue) {
  return [joystickValue]() {
    double deadband = frc::ApplyDeadband<double>(joystickValue(), 0.1);
    return std::abs(deadband) * deadband;
  };
}

// std::function<frc2::CommandPtr()> RobotContainer::GetAStarCmd() {
//   return [this] {
//     return driveSub.PathfindToSafeSpot(
//         [this] { return driveSub.CalculateClosestSafeSpot(); });
//   };
// }

