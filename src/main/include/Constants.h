// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <numbers>
#include <cmath>
/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 * 
 * 
 * 
 */



const double BALLEVATOR_SPEED_IDLE = 0.0;
const double BALLEVATOR_SPEED_READY = 0.75;
const double BALLEVATOR_SPEED_LOADING = 0.65;
const double BALLEVATOR_SPEED_HOLD = 0.0;
const double BALLEVATOR_SPEED_FIRE = 1.0;
const double BALLEVATOR_SPEED_REVERSE = -1.0;

const double TURRET_YAW_DEADZONE = 0.25;
const double TURRET_ANGLE_DEADZONE = 0.2;
const double TURRET_SPEED_DEADZONE = 0.2;

namespace Constants {
using std::numbers::pi;
namespace RobotContainer {
constexpr double minThrottle = .1;
}
namespace DriveTrain {
constexpr int drvnum[]{1, 4, 7, 2}, trnnum[]{0, 5, 6, 3},
    encodernum[]{0, 3, 2, 1};
constexpr double halfLen = 24.25 / 2, halfWid = 24.125 / 2;  // X is forward
constexpr double offsetXs[]{halfLen, halfLen, -halfLen,
                            -halfLen};  // coords in inches
constexpr double offsetYs[]{halfWid, -halfWid, -halfWid,
                            halfWid};  // right front is +, +
const double rotationRescale = std::sqrt(
    halfLen * halfLen +
    halfWid * halfWid);  // todo: de-kludge this, it makes the linear and
                         // rotational control argumenrs comparable.
constexpr double turnGearing = 72.0 / 14 * 24 / 12,  // maybe use std::ratio
    ticksPerRotation = turnGearing * 2048,
    velPerRPS = ticksPerRotation / 10, //velocity per rotation/sec = accel per rotation/sec^2
    ticksPerRadian = ticksPerRotation / 2 / pi;  // todo: check this with hardware
constexpr int absSubtraction[]{2220, 2148, 2695, 1023};  // to align the wheels
constexpr double perSecond = 4, // inversely proportional to the intended response time
                maxVel = perSecond * velPerRPS, maxAccel = perSecond * perSecond * velPerRPS;//for Motion Magic
constexpr double CtlP = 0.1, CtlF = 0.25*2048/maxAccel;//for PID
}  // namespace DriveTrain

namespace Intake {
constexpr int whnum{4};     /* mechanism that initiates intake mechanism*/
constexpr double whpow = -.3;  // default power for the motor whnum

}  // namespace Intake

// namespace for the arm (getting and shooting the ball)
namespace Arm {
    using Constants::pi;
constexpr int stage1Id = 9, stage2Id = 8, stage3Id = 10;
constexpr double gearing = 36, ticksPerRotation = gearing * 2048, velPerRPS = ticksPerRotation / 10,
        ticksPerRadian = ticksPerRotation / 2 / pi,
        gravCompensator = 9.81 /*metric gravity*/ /39.37 /*inch/meter*/ / 2.205 /*lb/kg*/ / 4.69 /*stall torque*/ / gearing, gear1to2 = 2;
constexpr double momentStage1 = 202.4, nAngleStage1 = 6.1/360,
                 momentStage2 = 109.9, nAngleStage2 = 26.2/360; //todo: real values
constexpr double maxVel = 2 * velPerRPS, maxAccel = .2 * velPerRPS;//for Motion Magic
// the Motion Magic parameters are translated in interanl units from rotations/sec and rotations/sec^2
constexpr double CtlP = 0.1, CtlF = 0.5 * 1024 / maxVel;//for PID
// limits of motion, per stage
constexpr double minDegrees[] = {-90 * ticksPerRotation / 360, 0 * ticksPerRotation / 360, 0 * ticksPerRotation / 360},
                 maxDegrees[] = {90 * ticksPerRotation / 360, 340 * ticksPerRotation / 360, 340 * ticksPerRotation / 360};
}  // namespace Arm

}  // namespace Constants