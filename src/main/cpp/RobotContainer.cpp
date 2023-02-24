// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>

#include "Constants.h"
using namespace Constants::RobotContainer;


RobotContainer::RobotContainer()
    : m_autonomousCommand(&intake), drivebyStick(drivetrain, *this) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
  drivetrain.SetDefaultCommand(drivebyStick);
  arm.SetDefaultCommand(turretbyStick);
  intake.SetDefaultCommand(Intakebystick);
}

void RobotContainer::Init() {
  drivetrain.Init();
  if (frc::DriverStation::GetJoystickIsXbox(0)) XBox = true;
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  //if(1) return new autoaim(arm, ballevator);
  return &m_autonomousCommand;
}

double RobotContainer::GetX() {
  if (XBox) return driverstick.GetLeftY();
  return drivestickJ.GetY();
}

double RobotContainer::GetY() {
  if (XBox) return -driverstick.GetLeftX();
  return -drivestickJ.GetX();
}

double RobotContainer::GetRot() {
  if (XBox) return driverstick.GetRightX();
  return drivestickJ.GetTwist();
}

double RobotContainer::GetThrottle() {
  if (XBox)
    return (minThrottle +
            driverstick.GetRightTriggerAxis() * (1 - minThrottle));
  return (1 + minThrottle - drivestickJ.GetThrottle() * (1 - minThrottle)) / 2;
}

bool RobotContainer::GetFieldCenterToggle() {
  if (XBox) return driverstick.GetRightBumperPressed();
  return drivestickJ.GetTriggerPressed();
}

  // Values determined empirically by wiggling the joystick
// around. If you find that it tends to "stick" when released
// or jolt around unexpectedly, you may need to increase these.
const double JOYSTICK_MOTION_DEADZONE = 0.1;
const double JOYSTICK_ROTATION_DEADZONE = 0.2;

// Joysticks are noisy inputs and you need to ignore everything
// below some threshold and just clamp it to zero. This function
// zeroes out small inputs (say, +/- <dz>) and rescales the ranges
// (-1.0, <dz>] and [<dz>, 1.0) to (-1.0, 0.0) and (0.0, 1.0) so
// that every possible signal can still be input outside of the
// dead zone.
double CalculateDeadZone(double deadzone, double x) {
  if (std::abs(x) < deadzone) {
    return 0;
  }
  if (x > 0) {
    x = (x - JOYSTICK_MOTION_DEADZONE) / (1.0 - JOYSTICK_MOTION_DEADZONE);
  } else {
    x = (x + JOYSTICK_MOTION_DEADZONE) / (1.0 - JOYSTICK_MOTION_DEADZONE);
  }
  return x;
}

// Square the input except preserving the input sign, so
// that for instance -0.5 becomes -0.25 rather than 0.25.
double QuadraticScaling(double x) { return (x < 0.0 ? -1.0 : 1.0) * x * x; }

void RobotContainer::DrvbyStick::Execute() {
      static bool fieldcentered = true;
      if (bot.GetFieldCenterToggle()) fieldcentered ^= true;
      // todo: add throttle
  double linX = -bot.GetX(),
        linY = bot.GetY(),
        rot = bot.GetRot();
  // Dead zones
  linX = CalculateDeadZone(JOYSTICK_MOTION_DEADZONE, linX);
  linY = CalculateDeadZone(JOYSTICK_MOTION_DEADZONE, linY);
  rot = CalculateDeadZone(JOYSTICK_ROTATION_DEADZONE, rot);

  // Quadratic scaling on inputs
  linX = QuadraticScaling(linX);
  linY = QuadraticScaling(linY);
  rot = QuadraticScaling(rot);
      it.SetV(linX, linY, rot, bot.GetThrottle(),
              fieldcentered);
    }

void RobotContainer::TurbyStick::Execute() {
  // testing only
  static bool setPos = false, hold = false;
  double x = joy.GetLeftX(), y = joy.GetRightX();
  //if (joy.GetAButtonPressed()) setPos = ! setPos;
  //if (joy.GetLeftStickButtonPressed()) hold = ! hold;
  if (setPos) {
    x *= 90; y *= 90;
    it.SetAngles(x, y, 0);
    if (! hold) {
      frc::SmartDashboard::PutNumber("stage 1 angle:", x);
      frc::SmartDashboard::PutNumber("stage 2 angle:", y);
    }
  }
  else {
    x /= 10; y /= 10;
    if (joy.GetXButton()){
      it.SetMotors(x, 0, 0);
      if (! hold) 
        frc::SmartDashboard::PutNumber("stage 1 power:", x);
    }    
    else if (joy.GetYButton()){
      it.SetMotors(0, x, 0);
      if (! hold) 
        frc::SmartDashboard::PutNumber("stage 2 power:", x);
    }    
    else if (joy.GetBButton()){
      it.SetMotors(0, 0, x);
      if (! hold) 
        frc::SmartDashboard::PutNumber("stage 3 power:", x);
    }
    else it.SetMotors(0, 0, 0); 
  }
}