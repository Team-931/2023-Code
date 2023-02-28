#include <frc/smartdashboard/SmartDashboard.h>

#include "AutoModeSelector.h"

AutoModeSelector::AutoModeSelector()
{
    m_modeChooser.SetDefaultOption("Do Nothing", DO_NOTHING);
    frc::SmartDashboard::PutData("Autonomous Selected", &m_modeChooser);
}