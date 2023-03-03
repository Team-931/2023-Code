// This will store the definitions of the Arm's functionalities stored in
// Arm.h

#include "subsystems/Arm.h"

#include <frc/smartdashboard/SmartDashboard.h>
/* #include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableValue.h>
 *///#include <wpi/span.h>

#include "Constants.h"

using namespace Constants::Arm;

Arm::Arm()
    : stage1(stage1Id),
      stage2(stage2Id),
      stage3(stage3Id) {
  //stage1.Follow(stage2);
  stage1.SetInverted(TalonFXInvertType::Clockwise);
  stage2.SetInverted(TalonFXInvertType::Clockwise);
  stage3.SetInverted(TalonFXInvertType::Clockwise);
  stage1.SetNeutralMode(Brake);
  stage2.SetNeutralMode(Brake);
  stage3.SetNeutralMode(Brake);
  stage1.Config_kP(0, CtlP);
  stage1.Config_kF(0, CtlF/gear1to2);
  stage1.ConfigMotionCruiseVelocity(maxVel*gear1to2);
  stage1.ConfigMotionAcceleration(maxAccel*gear1to2);
  stage2.Config_kP(0, CtlP);
  stage2.Config_kF(0, CtlF);
  stage2.ConfigMotionCruiseVelocity(maxVel);
  stage2.ConfigMotionAcceleration(maxAccel);
  stage3.Config_kP(0, CtlP);
  stage3.Config_kF(0, CtlF);
  stage3.ConfigMotionCruiseVelocity(maxVel);
  stage3.ConfigMotionAcceleration(maxAccel);
  stage1.ConfigReverseSoftLimitThreshold(minDegrees[0]);
  stage1.ConfigReverseSoftLimitEnable(true);
  stage2.ConfigReverseSoftLimitThreshold(minDegrees[1]);
  stage2.ConfigReverseSoftLimitEnable(true);
  stage3.ConfigReverseSoftLimitThreshold(minDegrees[2]);
  stage3.ConfigReverseSoftLimitEnable(true);
  stage1.ConfigForwardSoftLimitThreshold(maxDegrees[0]);
  stage1.ConfigForwardSoftLimitEnable(true, 50);
  stage2.ConfigForwardSoftLimitThreshold(maxDegrees[1]);
  stage2.ConfigForwardSoftLimitEnable(true, 50);
  stage3.ConfigForwardSoftLimitThreshold(maxDegrees[2]);
  stage3.ConfigForwardSoftLimitEnable(true, 50);
  stage1.SetSelectedSensorPosition(initialCorrections[0]);
  stage2.SetSelectedSensorPosition(initialCorrections[1]);
  stage3.SetSelectedSensorPosition(initialCorrections[2]);
}

void Arm::Periodic() {
  /* frc::SmartDashboard::PutNumber("shooterSpeed(actual)",
                                 stage2.GetSelectedSensorVelocity());
   */
  double theta1=stage1.GetSelectedSensorPosition() * 360 / ticksPerRotation / gear1to2;
  double theta2=stage2.GetSelectedSensorPosition() * 360 / ticksPerRotation;
  double theta3=stage3.GetSelectedSensorPosition() * 360 / ticksPerRotation;
  double s1 = theta1, s2 = theta2 - s1, s3 = theta3 - s2;
  frc::SmartDashboard::PutNumber("theta1", theta1);
  frc::SmartDashboard::PutNumber("theta2", theta2);
  frc::SmartDashboard::PutNumber("theta3", theta3);
  frc::SmartDashboard::PutNumber("s1", s1);
  frc::SmartDashboard::PutNumber("s2", s2);
  frc::SmartDashboard::PutNumber("s3", s3);
}

void Arm::SetAngles(double deg1, double deg2, double deg3) {
  deg1 /= 360; deg2 /= 360; deg3 /= 360;
  double g3 = gravCompensator * momentStage3 * sin (2*pi*(deg3 - nAngleStage3));//fix this
  double g2 = gravCompensator * momentStage2 * sin (2*pi*(deg2 - nAngleStage2)) + g3;
  double g1 = gravCompensator * momentStage1 * sin (2*pi*(deg1 - nAngleStage1)) + g2;
  stage1.Set(ControlMode::MotionMagic, deg1 * ticksPerRotation * gear1to2,
    DemandType::DemandType_ArbitraryFeedForward, g1 / gear1to2);
  stage2.Set(ControlMode::MotionMagic, (deg1 + deg2) * ticksPerRotation, 
    DemandType::DemandType_ArbitraryFeedForward, -g2);
  stage3.Set(ControlMode::MotionMagic, (deg2 + deg3) * ticksPerRotation, 
    DemandType::DemandType_ArbitraryFeedForward, g3);
}

void Arm::SetAngles(double (&angles)[3]) {
  SetAngles(angles[0], angles[1], angles[2]);
}

void Arm::SetVeloc(double vel1, double vel2, double vel3) {
  double deg1 = stage1.GetSelectedSensorPosition() / ticksPerRotation / gear1to2,
       deg2 = stage2.GetSelectedSensorPosition() / ticksPerRotation - deg1, 
       deg3 = stage3.GetSelectedSensorPosition() / ticksPerRotation - deg2;
  double g3 = gravCompensator * momentStage3 * sin (2*pi*(deg3 - nAngleStage3));//fix this
  double g2 = gravCompensator * momentStage2 * sin (2*pi*(deg2 - nAngleStage2)) + g3;
  double g1 = gravCompensator * momentStage1 * sin (2*pi*(deg1 - nAngleStage1)) + g2;
  stage1.Set(ControlMode::Velocity, vel1 * ticksPerRotation * gear1to2/10,
    DemandType::DemandType_ArbitraryFeedForward, g1 / gear1to2);
  stage2.Set(ControlMode::Velocity, (vel2 + vel1) * ticksPerRotation/10, 
    DemandType::DemandType_ArbitraryFeedForward, -g2);
  stage3.Set(ControlMode::Velocity, (vel3 + vel2) * ticksPerRotation/10, 
    DemandType::DemandType_ArbitraryFeedForward, g3);
}
void Arm::SetLinVeloc(double fwdInPerSec, double upInPerSec, double vel3)
{
  double  deg1 = stage1.GetSelectedSensorPosition() / ticksPerRotation / gear1to2,
          theta2 = stage2.GetSelectedSensorPosition() / ticksPerRotation,
          deg2 = theta2 - deg1, 
          deg3 = stage3.GetSelectedSensorPosition() / ticksPerRotation - deg2;
  double g3 = gravCompensator * momentStage3 * sin (2*pi*(deg3 - nAngleStage3));//fix this
  double g2 = gravCompensator * momentStage2 * sin (2*pi*(deg2 - nAngleStage2)) + g3;
  double g1 = gravCompensator * momentStage1 * sin (2*pi*(deg1 - nAngleStage1)) + g2;
  double vel1 = (sin(deg2) * fwdInPerSec + cos(deg2) * upInPerSec) * 24 / 21,
         vel2 = sin(deg1) * fwdInPerSec - cos(deg2) * upInPerSec;
  stage1.Set(ControlMode::Velocity, vel1 * ticksPerRotation * gear1to2/10,
    DemandType::DemandType_ArbitraryFeedForward, g1 / gear1to2);
  stage2.Set(ControlMode::Velocity, (vel2 + vel1) * ticksPerRotation/10, 
    DemandType::DemandType_ArbitraryFeedForward, -g2);
  stage3.Set(ControlMode::Velocity, (vel3 + vel2) * ticksPerRotation/10, 
    DemandType::DemandType_ArbitraryFeedForward, g3);
}
