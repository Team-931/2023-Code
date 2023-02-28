#pragma once

#include <frc/smartdashboard/SendableChooser.h>

class AutoModeSelector
{
public:
    AutoModeSelector();

    enum DesiredMode
    {
        DO_NOTHING
    };

private:
    frc::SendableChooser<DesiredMode> m_modeChooser;
};
