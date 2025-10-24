// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class DioLimitSwitch {
    private DigitalInput m_dio;
    private boolean m_inverted;
    public DioLimitSwitch(int dioPort, boolean inverted) {
        m_dio = new DigitalInput(dioPort);
        m_inverted = inverted;
    }

    public boolean get() {
        //Inverted sensors output 0 when triggered, XOR output with m_inverted
        return (m_dio.get() ^ m_inverted);
    }
}
