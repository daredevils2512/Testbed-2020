/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

/**
 * Wrapper class for standard ultrasonic sensor
 */
public class Ultrasonic {
  private final AnalogInput m_analogIn;
  private final AnalogPotentiometer m_analogPotentiometer;

  public Ultrasonic(int analogInPort) {
    m_analogIn = new AnalogInput(analogInPort);
    m_analogPotentiometer = new AnalogPotentiometer(m_analogIn, 1 / 9.77 / 1000);
  }

  public double getVoltage() {
    return m_analogIn.getVoltage();
  }

  public double getDistance() {
    return m_analogPotentiometer.get();
  }
}
