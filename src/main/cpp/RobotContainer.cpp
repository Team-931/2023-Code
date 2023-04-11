// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <frc2/command/SequentialCommandGroup.h>
# include <frc2/command/WaitCommand.h>
#include "RobotContainer.h"

#include <frc/DriverStation.h>

#include "Constants.h"
using namespace Constants::RobotContainer;

#include "commands/AutoDrive.h"

RobotContainer::RobotContainer()
    : m_autonomousCommand(&intake), drivebyStick(drivetrain, *this) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
  drivetrain.SetDefaultCommand(drivebyStick);
  arm.SetDefaultCommand(turretbyStick);
  intake.SetDefaultCommand(Intakebystick);
}

void RobotContainer::InitAlign() {
  drivetrain.Init();
}
void RobotContainer::Init() {
  autoChooser.SetDefaultOption("do nothing", 0);
  autoChooser.AddOption("move 16.5 ft", 1);
  autoChooser.AddOption("score and back 16.5 ft", 2);
  autoChooser.AddOption("play reveille", 4);
  frc::SmartDashboard::PutData(&autoChooser);
  if (frc::DriverStation::GetJoystickIsXbox(0))
    XBox = true;
}

struct PlayCommand : frc2::WaitCommand {
  PlayCommand() : frc2::WaitCommand (1_s) {}
  void Initialize() override {
    DriveTrain::PlayMusic();
  }
};

# include "armVectors.inc"

class testarmraise : public /* frc2::CommandHelper< */frc2::WaitCommand/* , testarmraise> */ {
  public:
   testarmraise (Arm& a, const double (&angles)[3], units::second_t timeout = 15_s) : WaitCommand(timeout), arm(a), angs(angles)
       {
    AddRequirements (&a);
   }
   void Initialize() override {
    arm.SetAngles(angs);
    timesInRange = 0;
   }
   void Execute() override;
   bool IsFinished() override;

  private:
   Arm& arm;
   const double (&angs)[3];
   int timesInRange = 0;
   static const int minTimesInRange = 10;
};

void testarmraise::Execute() {
  if (arm.AtSetpoint(angs)) ++ timesInRange;
  else timesInRange = 0;
}

bool testarmraise::IsFinished() {
  //return false;
  return frc::DriverStation::IsDisabled() ||
    (timesInRange >= minTimesInRange);
}

struct IntCtl
      : public frc2::WaitCommand {
    IntCtl(Intake& i, IntakeState s) : WaitCommand(1_s), it(i), state(s) {
      AddRequirements(&i);
    }
    void Execute() override {
      // Control robot
      it.SetDeployed(state);
    }
    void End(bool) override {
      it.SetDeployed(Stop);
    }
    Intake& it;
    IntakeState state = Stop;
  };

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  switch (autoChooser.GetSelected()) {
  case 0: return 0;
  case 1:
   return new AutoDrive(drivetrain, 13_ft, 0_ft, .3);
  case 2:
   return new frc2::SequentialCommandGroup(
    testarmraise(arm, openInFront),
    testarmraise(arm, highPost),
    IntCtl(intake, ConeIn),
    testarmraise(arm, openInFront),
    testarmraise(arm, foldedDown, 1_s),
    AutoDrive(drivetrain, -13_ft, 0_ft, .3)
    );
  case 4:
   return new PlayCommand;
  }
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
  if (XBox) {
    double lefTrig = driverstick.GetLeftTriggerAxis(),
           righTrig = driverstick.GetRightTriggerAxis();
    if (lefTrig < stickError)
      return baseThrottle +
            righTrig * (maxThrottle - baseThrottle);
    else
    if (righTrig < stickError)
      return baseThrottle + lefTrig * (minThrottle - baseThrottle);
    else return baseThrottle;
  }
  return (maxThrottle + minThrottle - drivestickJ.GetThrottle() * (maxThrottle - minThrottle)) / 2;
}
# ifdef FdCtrTog
bool RobotContainer::GetFieldCenterToggle() {
  if (XBox) return driverstick.GetRightBumperPressed();
  return drivestickJ.GetTriggerPressed();
}
# endif
bool RobotContainer::GetZeroYaw() {
  if (XBox) return driverstick.GetAButton();
  return drivestickJ.GetTop();
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
# ifdef FdCtrTog
      if (bot.GetFieldCenterToggle()) fieldcentered ^= true;
# endif     
      if (bot.GetZeroYaw()) it.ZeroYaw();
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
  if (frc::DriverStation::IsDisabled()) {
    setPos = false;
    it.SetMotors(0, 0, 0);
    return;
  }
  if (joy.GetAButton()) {
    it.SetAngles(openInFront);
    setPos = true;
  }
  if (joy.GetRightBumper()) {
    it.SetAngles(foldedDown);
    setPos = true;
  }
  if (joy.GetLeftBumper()) {
    it.SetAngles(cubeOnFloor);
    setPos = true;
  }
  if (joy.GetYButton()) {
    it.SetAngles(highPost);
    setPos = true;
  }
  if (joy.GetXButton()) {
    it.SetAngles(lowPost);
    setPos = true;
  }
  if (joy.GetBButton()) {
    it.SetAngles(conePickup1);
    setPos = true;
  }
  {
    int pov = joy.GetPOV();
    if (pov == 180) {
      it.SetAngles(openInBack);
      setPos = true;
    }
    if (pov == 90) {
      it.SetAngles(lowPostBack);
      setPos = true;
    }
    if (pov == 270) {
      it.SetAngles(coneOnFloorBack1);
      setPos = true;
    }
  }
  /* if (joy.GetYButton() && joy.GetRightBumper()) {
    it.SetAngles(coneOnFloor);
    setPos = true;
  }
  if (joy.GetYButton() && joy.GetLeftBumper()) {
    it.SetAngles(cubeOnFloor);
    setPos = true;
  }
 */
  double y = joy.GetRightY();
    /*  x /= 10; y /= 10;
    if (joy.GetXButton()){
      it.SetVeloc(x, 0, 0);
      setPos = false;
    }
    else if (joy.GetYButton()){
      it.SetVeloc(0, x, 0);
      setPos = false;
    }
    else if (joy.GetBButton()){
      it.SetVeloc(0, 0, x);
      setPos = false;
    } */
    if (abs(y) >= stickError) {
      it.HoldStill(-y * .02 /*ms*/ * 40 / (2 * pi * len3));
      setPos = true;
    }
    else if (setPos == false) {
      it.HoldStill();
      setPos = true;
    }
 }