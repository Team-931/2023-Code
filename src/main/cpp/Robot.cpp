#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

#include "Robot.h"

void Robot::RobotInit()
{
}

/**
 * This function is called periodically regardless of the mode that is running
 */
void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
}

/**
 * Called once when the robot becomes disabled
 */
void Robot::DisabledInit()
{
}

/**
 * Called periodically while the robot is disabled
 */
void Robot::DisabledPeriodic()
{
}

/**
 * Called once when an autonomous routine is started
 */
void Robot::AutonomousInit()
{
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand != nullptr)
    {
        m_autonomousCommand->Schedule();
    }

    m_container.Init(); // setting the encoders based on absolute readings: does
                        // it work here?
}

/**
 * Called periodically while the robot is in autonomous
 */
void Robot::AutonomousPeriodic()
{
}

/**
 * Called periodically while the robot is in teleop
 */
void Robot::TeleopInit()
{
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != nullptr)
    {
        m_autonomousCommand->Cancel();
        m_autonomousCommand = nullptr;
    }

    m_container.Init(); // setting the encoders based on absolute readings: does
                        // it work here?
}

/**
 * Called periodically while the robot is in teleop
 */
void Robot::TeleopPeriodic()
{
}

/**
 * Called once when test mode is started
 */
void Robot::TestInit()
{
}

/**
 * Called periodically while the robot is in test mode
 */
void Robot::TestPeriodic()
{
}

/**
 * Called once when simulation mode is started
 */
void Robot::SimulationInit()
{
}

/**
 * Called periodically while the robot is in simulation mode
 */
void Robot::SimulationPeriodic()
{
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
