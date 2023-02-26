// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

#include "Constants.h"
using namespace Constants::Intake;

Intake::Intake()
    : wheels(whnum, rev::CANSparkMax::MotorType::kBrushless),
      deployed(Stop) {
  wheels.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Intake::Periodic() {
    wheels.Set(whpow * (int) deployed);
}

void Intake::SetDeployed(IntakeState d) { deployed = d; }

IntakeState Intake::IsDeployed() { return deployed; }

void Intake::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
