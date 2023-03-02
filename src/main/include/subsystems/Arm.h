// This will store functionalities for the turret on the bot
#include <ctre/Phoenix.h>
#pragma once

#include <frc2/command/SubsystemBase.h>

class Arm : public frc2::SubsystemBase {
 public:
  Arm();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetAngles(double stage1Degrees, double stage2Degrees, double stage3Degrees); // 0, 0 is start position 
  void SetVeloc(double stage1DegreesPerSec, double stage2DegreesPerSec, double stage3DegreesPerSec); 
  
  void SetMotors(double st1, double st2, double st3){
    stage1.Set(st1); stage2.Set(st2); stage3.Set(st3);// testing only
  }
 private:
  WPI_TalonFX stage1, stage2, stage3;
};