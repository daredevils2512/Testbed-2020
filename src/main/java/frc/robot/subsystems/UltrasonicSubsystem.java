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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicSubsystem extends SubsystemBase {
  private final NetworkTable m_networkTable;

  private final int m_analogInChannel = 0;
  private final AnalogInput m_analogIn;

  /**
   * Creates a new ultrasonic subsystem
   */
  public UltrasonicSubsystem() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());

    m_analogIn = new AnalogInput(m_analogInChannel);
  }

  @Override
  public void periodic() {
    m_networkTable.getEntry("Distance").setDouble(m_analogIn.getVoltage() * 1000 / 9.77);
  }
}
