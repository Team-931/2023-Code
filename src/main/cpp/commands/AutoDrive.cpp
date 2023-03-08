# include "commands/AutoDrive.h"
# include <units/math.h>

AutoDrive::AutoDrive(DriveTrain &subsystem, units::inch_t fwd, units::inch_t left, double motorPwr) : drive(subsystem)
{
    AddRequirements(&drive);
    dist1 = units::math::sqrt(fwd*fwd + left*left);
    auto mP = motorPwr / dist1;
    x = fwd * mP;
    y = left * mP;
}

void AutoDrive::Initialize()
{
    drive.SetV(x, y, 0, 1);
    startEnc = drive.GetDistance();
}

void AutoDrive::Execute()
{
}

bool AutoDrive::IsFinished()
{
    return units::math::abs(startEnc - drive.GetDistance()) >= dist1;
}
