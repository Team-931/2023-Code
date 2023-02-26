// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#define DRIVE_TRAIN_DEBUG
#include "subsystems/DriveTrain.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>
#include <string>

#include "Constants.h"
using namespace Constants::DriveTrain;

DriveTrain::DriveTrain() {
  // Implementation of subsystem constructor goes here.
  SetName("drive train");
  symph.LoadMusic("startup.chrp");
}

void DriveTrain::SetV(double linX, double linY, double rot, double throttle,
                      bool fieldctr) {
#ifdef DRIVE_TRAIN_DEBUG
  frc::SmartDashboard::PutNumber("linX", linX);
  frc::SmartDashboard::PutNumber("linY", linY);
  frc::SmartDashboard::PutNumber("rot", rot);
#endif /* DRIVE_TRAIN_DEBUG */
  if (fieldctr) {
    // todo:
    double yaw = std::numbers::pi / 180 * navx.GetYaw();
    double c = std::cos(yaw), s = std::sin(yaw);
    double x = linX * c - linY * s, y = linX * s + linY * c;
    linX = x;
    linY = y;
  }

  double scale = 1 / throttle;
  for (auto& wheel : wheels)
    scale = std::max(scale, wheel.SetV(linX, linY, rot));
  for (auto& wheel : wheels) wheel.ScaleV(scale);
}

void DriveTrain::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void DriveTrain::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
Orchestra symph;
int SwerveModule::ix = 0;

SwerveModule::SwerveModule()
    : drive(drvnum[ix]),
      turn(trnnum[ix]),
      absAngle(encodernum[ix]),
      offsetX(offsetXs[ix]),
      offsetY(offsetYs[ix]),
      index(ix) {
  SetName("wheels " + std::to_string(ix));
  AddChild("absAngle", &absAngle);
  ++ix;
  turn.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
  turn.SetNeutralMode(NeutralMode::Coast);
  // does this work?
      turn.ConfigIntegratedSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
      turn.ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Signed_PlusMinus180);
      // set PID
  turn.Config_kP(0, CtlP);
  turn.Config_kF(0, CtlF);
  turn.ConfigMotionCruiseVelocity(maxVel);
  turn.ConfigMotionAcceleration(maxAccel);
  drive.SetNeutralMode(NeutralMode::Brake);
  drive.ConfigOpenloopRamp(0);//ask how much
  drive.ConfigClosedloopRamp(.5);//ask how much
  symph.AddInstrument(turn);
  symph.AddInstrument(drive);
}

double SwerveModule::SetV(double linX, double linY, double rot) {
  rot /= rotationRescale;
  linX += offsetY * rot;
  linY -= offsetX * rot;
  double spd = speed = std::sqrt(linX * linX + linY * linY),
         ang = -std::atan2(linY, linX);//this is a kludge
  // this puts angle into same semicircle as its old value,
  // equivalent to adding or subtracting multiples of 180 degrees to ang to keep
  // it as close to angle as possible. Note that if we change ang by an odd
  // number of semicircles we change the sign of speed.
  int phase;
  angle = angle - std::remquo(angle - ang, std::numbers::pi, &phase);
  if ((phase & 1) == 0) speed = -speed;
  // double oldangle = turn.GetSelectedSensorPosition();//maybe later if we want
  // to refer to actual position
  turn.Set(ControlMode::MotionMagic, ticksPerRadian * angle);
  return spd;
}

void SwerveModule::ScaleV(double scale) {
  speed /= scale;  // we assert scale >= 1
                   // frc::SmartDashboard::PutNumber ("speed", speed);
}

void SwerveModule::Periodic() {
  // Implementation of subsystem periodic method goes here.
  drive.Set(speed);
#ifdef DRIVE_TRAIN_DEBUG
  static int ctr = 0;
  if ((ctr++) % 5 == 0) {
    double ang = absAngle.GetAbsolutePosition();
    frc::SmartDashboard::PutNumber(GetName() + " abs Encoder", 4096 * ang);
    frc::SmartDashboard::PutNumber(
        GetName() + " encoder diff",
        turn.GetSelectedSensorPosition() + ticksPerRotation * ang);
  }
#endif /* DRIVE_TRAIN_DEBUG */
}
void DriveTrain::Init() {
  for (auto& wheel : wheels) wheel.Init();
  symph.Play();
}

void SwerveModule::Init() {
  turn.SetSelectedSensorPosition(
      ticksPerRotation *
      (absSubtraction[index] / 4096. - absAngle.GetAbsolutePosition()));
}

void SwerveModule::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}