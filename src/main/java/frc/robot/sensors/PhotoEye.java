/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Wrapper class for photo switches
 */
public class PhotoEye {
  private DigitalInput m_digitalIn;

  public PhotoEye(int port) {
    m_digitalIn = new DigitalInput(port);
  }

  public boolean getDetected() {
    return m_digitalIn.get();
  }
}
