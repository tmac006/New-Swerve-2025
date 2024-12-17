#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/PrintCommand.h"
#include "subsystems/DrivebaseSubsystem.h"
#include "subsystems/ExampleSubsystem.h"

namespace autos {
/**
 * Example static factory for an autonomous command.
 */
frc2::CommandPtr ExampleAuto(ExampleSubsystem* subsystem);
}  // namespace autos
