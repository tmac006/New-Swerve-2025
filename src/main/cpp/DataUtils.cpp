#include "DataUtils.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/Filesystem.h>

#include <fstream>
#include <sstream>

void DataUtils::SetupDataLogging() {
  // These lines of code will record all changes to network table values as well
  // as driverstation joystick and control values
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
}

void DataUtils::LogGitInfo() {
  std::string branchFileName =
      frc::filesystem::GetDeployDirectory() + "/" + "branch.txt";

  std::ifstream branchFile(branchFileName);
  if (branchFile.fail()) {
    frc::DataLogManager::Log(fmt::format("Error opening branch file!"));
  }

  std::stringstream branchStream;
  branchStream << branchFile.rdbuf();

  std::string commitFileName =
      frc::filesystem::GetDeployDirectory() + "/" + "commit.txt";
  std::ifstream commitFile(commitFileName);
  if (commitFile.fail()) {
    frc::DataLogManager::Log(fmt::format("Error opening branch file!"));
  }

  std::stringstream commitStream;
  commitStream << commitFile.rdbuf();

  frc::DataLogManager::Log(
      fmt::format("Robot is running code on branch {} with commit {}",
                  branchStream.str(), commitStream.str()));
}