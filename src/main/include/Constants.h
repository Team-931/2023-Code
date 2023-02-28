#pragma once
#include <cmath>
#include <numbers>
using std::numbers::pi;

/**
 * The Constants namespace provides a good place for robot-wide constant variables like port
 * numbers, drive measurements, and subsystem configurations. This can also be used to hold
 * numerical or boolean constants. This should not be used to hold any dynamic code.
 *
 * It is generally a good idea to place constants into subsystem-specific  namespaces within this
 * header, which can then be used where they are needed.
 */
namespace Constants
{
    namespace RobotContainer
    {
        constexpr double minThrottle = .1;
    }

    namespace DriveTrain
    {
        // Port number constants
        constexpr int drvnum[]{1, 4, 7, 2}, trnnum[]{0, 5, 6, 3}, encodernum[]{0, 3, 2, 1};

        // Drive train measurements (inches)
        constexpr double driveLength = 24.25, driveWidth = 24.125;
        constexpr double halfLen = driveLength / 2, halfWid = driveWidth / 2; // X is forward
        constexpr double offsetXs[]{halfLen, halfLen, -halfLen, -halfLen};
        constexpr double offsetYs[]{halfWid, -halfWid, -halfWid, halfWid};
        const double rotationRescale =
            std::sqrt(std::pow(halfLen, 2) +
                      std::pow(halfWid, 2)); // todo: de-kludge this, it makes the linear and
                                             // rotational control argumenrs comparable.
        constexpr double turnGearing = 72.0 / 14 * 24 / 12, ticksPerRotation = turnGearing * 2048,
                         velPerRPS = ticksPerRotation /
                                     10, // velocity per rotation/sec = accel per rotation/sec^2
            ticksPerRadian = ticksPerRotation / 2 / pi;         // todo: check this with hardware
        constexpr int absSubtraction[]{2220, 2148, 2695, 1023}; // to align the wheels
        constexpr double perSecond = 4, // inversely proportional to the intended response time
            maxVel = perSecond * velPerRPS,
                         maxAccel = perSecond * perSecond * velPerRPS; // for Motion Magic
        constexpr double CtlP = 0.1, CtlF = 0.25 * 2048 / maxAccel;    // for PID
    }                                                                  // namespace DriveTrain

    namespace Intake
    {
        constexpr int whnum{4};       /* mechanism that initiates intake mechanism*/
        constexpr double whpow = -.3; // default power for the motor whnum

    }

    namespace Arm
    {
        using Constants::pi;
        constexpr int stage1Id = 9, stage2Id = 8, stage3Id = 10;
        constexpr double gearing = 36, ticksPerRotation = gearing * 2048,
                         velPerRPS = ticksPerRotation / 10,
                         ticksPerRadian = ticksPerRotation / 2 / pi,
                         gravCompensator = 9.81 /*metric gravity*/ / 39.37 /*inch/meter*/ /
                                           2.205 /*lb/kg*/ / 4.69 /*stall torque*/ / gearing,
                         gear1to2 = 2;
        constexpr double momentStage1 = 202.4, nAngleStage1 = 6.1 / 360, momentStage2 = 109.9,
                         nAngleStage2 = 26.2 / 360;                         // todo: real values
        constexpr double maxVel = 2 * velPerRPS, maxAccel = .2 * velPerRPS; // for Motion Magic
        // the Motion Magic parameters are translated in interanl units from rotations/sec and
        // rotations/sec^2
        constexpr double CtlP = 0.1, CtlF = 0.5 * 1024 / maxVel; // for PID

    }
}