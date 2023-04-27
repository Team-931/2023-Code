// This will store functionalities for the turret on the bot
#include <ctre/Phoenix.h>
#pragma once
void makeAngles(double fwdIn, double htIn,
                        double (&output)[3],  
                        bool reverseElbow = false);
void asHighAsItGets(double fwdIn, double (&output)[3]);
double forwardDist(const double (&angles)[3]);
bool elbowReversed(const double (&angles)[3]);
#include <frc2/command/SubsystemBase.h>

class Arm : public frc2::SubsystemBase {
 public:
  Arm();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetAngles(double stage1Degrees, double stage2Degrees, double stage3Degrees); // 0, 0 is start position
  void SetAngles(const double (&angles)[3]); 
  void GetAngles(double (&angles)[3]);//over-writes the angles array
  bool AtSetpoint(const double (&angles)[3]);
  void SetVeloc(double stage1RotPerSec, double stage2RotPerSec, double stage3RotPerSec); 
  void SetLinVeloc(double fwdInPerSec, double upInPerSec, double stage3DegreesPerSec);
  void HoldStill(double mov3 = 0);
  void SetMotors(double st1, double st2, double st3){
    stage1.Set(st1); stage2.Set(st2); stage3.Set(st3);// testing only
  }
 private:
  WPI_TalonFX stage1, stage2, stage3;
};