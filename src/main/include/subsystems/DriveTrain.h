// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <frc/DutyCycleEncoder.h>
#include <frc2/command/SubsystemBase.h>

/**
 * All the control objects for a single wheel. Initialized with an index to Id.
 * tables in constants.h.
 */
class SwerveModule : public frc2::SubsystemBase {
 public:
  SwerveModule();

  /**
   * the module's motion is set relative to the robot's axes
   * SetV calculates an initial speed and angle, returns the speed.
   * ScaleV divides that speed by a scale factor to keep each wheels speed in
   * proportion and in range.
   */
  double SetV(double linX, double linY, double rot);
  void ScaleV(double scale);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Init();

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix::motorcontrol::can::WPI_TalonFX drive, turn;
  frc::DutyCycleEncoder absAngle;
  double offsetX, offsetY, speed{0}, angle;
  int index;
  static int ix;
};

/**
 * all the driving and navigation objects, incl. a SwerveModule for each wheel.
 */
class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();

  // X is forward
  /**
   * Set linear and rotational velocity relative to the field by default,
   * otherwise relative to the robot.
   * linX is forward
   * linY is rightward
   * rot is clockwise
   */
  void SetV(double linX, double linY, double rot, double throttles,
            bool fieldcentered = true);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Init();

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  SwerveModule wheels[4];
  AHRS navx{frc::SPI::Port::kMXP};
};
