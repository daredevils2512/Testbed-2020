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

  private final int m_dioPort1 = 1;
  private final int m_dioPort2 = 2;
  private final int m_dioPort3 = 3;
  private final int m_dioPort4 = 4;
  private final PhotoEye m_photoEye1;
  private final PhotoEye m_photoEye2;
  private final PhotoEye m_photoEye3;
  private final PhotoEye m_photoEye4;

  /**
   * Creates a new PhotoEyeSubsystem.
   */
  public PhotoEyeSubsystem() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());

    m_photoEye1 = new PhotoEye(m_dioPort1);
    m_photoEye2 = new PhotoEye(m_dioPort2);
    m_photoEye3 = new PhotoEye(m_dioPort3);
    m_photoEye4 = new PhotoEye(m_dioPort4);
  }

  @Override
  public void periodic() {
    m_networkTable.getEntry("Photo eye").setBoolean(getBallInQueue()[0]);
    m_networkTable.getEntry("Photo eye").setBoolean(getBallInQueue()[1]);
    m_networkTable.getEntry("Photo eye").setBoolean(getBallInQueue()[2]);
    m_networkTable.getEntry("Photo eye").setBoolean(getBallInQueue()[3]);
  }

  public boolean[] getBallInQueue() {
    boolean[] getBall = {};
    getBall[0] = !m_photoEye1.getDetected();
    getBall[1] = !m_photoEye2.getDetected();
    getBall[2] = !m_photoEye3.getDetected();
    getBall[3] = !m_photoEye4.getDetected();
    return getBall;
  }
}
