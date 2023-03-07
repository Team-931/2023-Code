# include "commands/AutoDrive.h"

AutoDrive::AutoDrive(DriveTrain &subsystem, double fwd, double left, double motorPwr) : drive(subsystem)
{
    AddRequirements(&drive);
    dist1 = sqrt(fwd*fwd + left*left);
    motorPwr /= dist1;
    x = fwd * motorPwr;
    y = left * motorPwr;
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
    return abs(startEnc - drive.GetDistance()) >= dist1;
}
