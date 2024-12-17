#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <DataUtils.h>
#include <wpinet/PortForwarder.h>
#include <iostream>

void Robot::RobotInit() 
{
   std::cout << "hello world\n";
  AddPeriodic([this] { m_container.GetDrivebaseSubsystem().UpdateOdometry(); },
              1 / 250_Hz);

pdh.ClearStickyFaults();
  // wpi::PortForwarder::GetInstance().Add(5800, "10.20.53.54", 5800); //fix later
  // wpi::PortForwarder::GetInstance().Add(5800, "10.20.53.55", 5800);
}

void Robot::RobotPeriodic() 
{
  frc2::CommandScheduler::GetInstance().Run();
  
  frc::Pose2d robot2dPose = m_container.GetDrivebaseSubsystem().GetRobotPose();
  frc::Rotation3d robot3drot{0_rad, 0_rad, robot2dPose.Rotation().Radians()};
}

// void Robot::SimulationPeriodic() {
//   m_container.GetVisionSystem().SimPeriodic(
//       m_container.GetDrivebaseSubsystem().GetOdomPose());
// }

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() 
{
   m_container.desiredAngle =
      m_container.GetDrivebaseSubsystem().GetRobotPose().Rotation().Radians();
}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {

  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
