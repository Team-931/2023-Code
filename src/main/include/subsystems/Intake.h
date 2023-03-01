
#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/cansparkmax.h>

enum IntakeState {Stop = 0, ConeIn = 1, CubeIn = -1, CubeShoot = 2};

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetDeployed(IntakeState deployed);
  IntakeState IsDeployed();

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax wheels;
  IntakeState deployed;
};
