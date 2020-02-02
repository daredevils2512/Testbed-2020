/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

public class GyroData {
  private double m_yaw;
  private double m_pitch;
  private double m_roll;

  public GyroData() {

  }

  public GyroData(double yaw, double pitch, double roll) {
    m_yaw = yaw;
    m_pitch = pitch;
    m_roll = roll;
  }

  public void setYaw(double yaw) {
    m_yaw = yaw;
  }

  public void setPitch(double pitch) {
    m_pitch = pitch;
  }

  public void setRoll(double roll) {
    m_roll = roll;
  }

  public double getYaw() {
    return m_yaw;
  }

  public double getPitch() {
    return m_pitch;
  }

  public double getRoll() {
    return m_roll;
  }
}
