// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc/XboxController.h>  //this is the file containing connection to xbox controller
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Command.h>

#include "commands/ExampleCommand.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/Intake.h"
#include "subsystems/Arm.h"
#include "Constants.h"

#define BALLEVATOR_IDLE 0
#define BALLEVATOR_READY 1
#define BALLEVATOR_LOADING 2
#define BALLEVATOR_HOLD 3
#define BALLEVATOR_FIRE 4
#define BALLEVATOR_REVERSE 5


/* HACK(wgd): This should probably live somewhere more 'utilities-ish'
   and not just be forward-declared here, but this is faster. */
double CalculateDeadZone(double deadzone, double x);

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  void Init();

  frc2::Command* GetAutonomousCommand();

  // get forward/back from the selected joystick in our coords
  double GetX();
  // get right/left from the selected joystick in our coords
  double GetY();
  // get rotation from the selected joystick in our coords
  double GetRot();
  // get throttle from the selected joystick in our coords
  double GetThrottle();
  //
  # ifdef FdCtrTog
bool GetFieldCenterToggle();
  # endif
  bool GetZeroYaw();

 private:
  // The robot's subsystems and commands are defined here...
  Intake intake;
  DriveTrain drivetrain;
  Arm arm;
  bool XBox{false};
  ExampleCommand m_autonomousCommand;

  void ConfigureButtonBindings();

struct DrvbyStick
      : public frc2::CommandHelper<frc2::CommandBase, DrvbyStick> {
    DrvbyStick(DriveTrain& d, RobotContainer& j) : it(d), bot(j) {
      AddRequirements(&d);
    }
    void Execute() override ;
    DriveTrain& it;
    RobotContainer& bot;
  } drivebyStick;

  struct TurbyStick
      : public frc2::CommandHelper<frc2::CommandBase, TurbyStick> {
    TurbyStick(Arm& t, frc::XboxController& j) : it(t), joy(j) {
      AddRequirements(&t);
          }
    void Execute() override; 
    bool RunsWhenDisabled() const override {return true;}
    Arm& it;
    frc::XboxController& joy;
    bool setPos{false};
  } turretbyStick{arm, operatorstick};

  struct IntbyStick
      : public frc2::CommandHelper<frc2::CommandBase, IntbyStick> {
    IntbyStick(Intake& i, frc::XboxController& j) : it(i), joy(j) {
      AddRequirements(&i);
    }
    void Execute() override {
      // State Variables
      static IntakeState intake_deployed = Stop;
      frc::SmartDashboard::PutBoolean("intake_deployed", intake_deployed);

      // Operator Inputs
      double x = joy.GetLeftY();
      bool intake_cone = x > .3;// ?? Pulling back
      bool intake_cube = x < .3;// ?? Pushing fwd

      // State update logic
      if (intake_cone) 
        if (intake_cube) intake_deployed = CubeShoot;//can't happen now!!
        else intake_deployed = ConeIn;
      else 
        if (intake_cube) intake_deployed = CubeIn;
        else intake_deployed = Stop;

      // Control robot
      it.SetDeployed(intake_deployed);
    }
    Intake& it;
    frc::XboxController& joy;
  } Intakebystick{intake, operatorstick};


  // The driver's controller (for manual control)
  frc::XboxController driverstick{
      0};  // 0 is only temporary (controller responsible for moving the robot)
  frc::Joystick drivestickJ{0};

  frc::XboxController operatorstick{
      1};  // 1 is only temporary (controller responsible for shooting the ball)
};
