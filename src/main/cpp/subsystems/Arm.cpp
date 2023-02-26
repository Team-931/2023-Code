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
  stage2.SetInverted(TalonFXInvertType::CounterClockwise);
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
  stage1.SetSelectedSensorPosition(initialCorrections[0]);
  stage2.SetSelectedSensorPosition(initialCorrections[1]);
  stage2.SetSelectedSensorPosition(initialCorrections[2]);
  stage1.ConfigReverseSoftLimitThreshold(minDegrees[0]);
  stage1.ConfigReverseSoftLimitEnable(true);
  stage2.ConfigReverseSoftLimitThreshold(minDegrees[1]);
  stage2.ConfigReverseSoftLimitEnable(true);
  stage3.ConfigReverseSoftLimitThreshold(minDegrees[2]);
  stage3.ConfigReverseSoftLimitEnable(true);
  stage1.ConfigForwardSoftLimitThreshold(maxDegrees[0]);
  stage1.ConfigForwardSoftLimitEnable(true);
  stage2.ConfigForwardSoftLimitThreshold(maxDegrees[1]);
  stage2.ConfigForwardSoftLimitEnable(true);
  stage3.ConfigForwardSoftLimitThreshold(maxDegrees[2]);
  stage3.ConfigForwardSoftLimitEnable(true);
}

void Arm::Periodic() {
  /* frc::SmartDashboard::PutNumber("shooterSpeed(actual)",
                                 stage2.GetSelectedSensorVelocity());
   */
}

void Arm::SetAngles(double deg1, double deg2, double deg3) {
  deg1 /= 360; deg2 /= 360; deg3 /= 360;
  double g3 = 0 /* gravCompensator * momentStage2 * sin (2*pi*(deg3 - nAngleStage2)) */;//fix this
  double g2 = gravCompensator * momentStage2 * sin (2*pi*(deg2 - nAngleStage2));
  double g1 = gravCompensator * momentStage1 * sin (2*pi*(deg1 - nAngleStage1)) + g2;
  stage1.Set(ControlMode::MotionMagic, deg1 * ticksPerRotation * gear1to2,
    DemandType::DemandType_ArbitraryFeedForward, g1 / gear1to2);
  stage2.Set(ControlMode::MotionMagic, (deg1 + deg2) * ticksPerRotation, 
    DemandType::DemandType_ArbitraryFeedForward, -g2);
  stage3.Set(ControlMode::MotionMagic, (deg2 + deg3) * ticksPerRotation, 
    DemandType::DemandType_ArbitraryFeedForward, g3);
}
