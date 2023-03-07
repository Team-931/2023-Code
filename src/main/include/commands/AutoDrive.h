
#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveTrain.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */

class AutoDrive
    : public frc2::CommandHelper<frc2::CommandBase, AutoDrive> {
 public:
  /**
   * Creates a new AutoDrive.
   *
   * @param subsystem The subsystem used by this command.
   */
  AutoDrive(DriveTrain& subsystem, double fwdDist, double leftDist, double motorPwr);

  void Initialize() override;

  void Execute() override;
  bool IsFinished() override;

 private:
  DriveTrain& drive;
  double startEnc, x, y, dist1;
};
