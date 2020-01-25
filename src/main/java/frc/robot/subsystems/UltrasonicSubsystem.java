/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicSubsystem extends SubsystemBase {
  private final NetworkTable m_networkTable;

  private final int m_analogInChannel = 0;
  private final int m_digitalOutChannel = 0;
  private final AnalogInput m_analogIn;
  private final DigitalOutput m_digitalOut;

  /**
   * Creates a new ultrasonic subsystem
   */
  public UltrasonicSubsystem() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());

    m_analogIn = new AnalogInput(m_analogInChannel);
    m_digitalOut = new DigitalOutput(m_digitalOutChannel);
  }

  @Override
  public void periodic() {
    m_networkTable.getEntry("Distance").setDouble(m_analogIn.getVoltage() * 1000 / 9.77);
  }
}
