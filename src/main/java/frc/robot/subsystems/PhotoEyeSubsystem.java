/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.PhotoEye;

public class PhotoEyeSubsystem extends SubsystemBase {
  private final NetworkTable m_networkTable;

  private final int m_dioPort = -1;
  private final PhotoEye m_photoEye;

  /**
   * Creates a new PhotoEyeSubsystem.
   */
  public PhotoEyeSubsystem() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());

    m_photoEye = new PhotoEye(m_dioPort);
  }

  @Override
  public void periodic() {
    m_networkTable.getEntry("Photo eye").setBoolean(m_photoEye.getDetected());
  }
}
