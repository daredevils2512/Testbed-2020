/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final int m_leftDriveMasterID = 1;
  private final int m_leftDrive1ID = 2;
  private final int m_rightDriveMasterID = 4;
  private final int m_rightDrive1ID = 6;

  private final WPI_TalonSRX m_leftDriveMaster;
  private final WPI_TalonSRX m_leftDrive1;
  private final WPI_TalonSRX m_rightDriveMaster;
  private final WPI_TalonSRX m_rightDrive1;

  private final DifferentialDrive m_differentialDrive;
  private final int m_encoderResolution = 4096; // Just a guess
  private final double m_gearRatio = 4 / 1;
  private final double m_wheelCircumference = 4 * Math.PI;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    m_leftDriveMaster = new WPI_TalonSRX(m_leftDriveMasterID);
    m_leftDrive1 = new WPI_TalonSRX(m_leftDrive1ID);
    m_rightDriveMaster = new WPI_TalonSRX(m_rightDriveMasterID);
    m_rightDrive1 = new WPI_TalonSRX(m_rightDrive1ID);

    m_leftDrive1.follow(m_leftDriveMaster);
    m_rightDrive1.follow(m_rightDriveMaster);

    m_differentialDrive = new DifferentialDrive(m_leftDriveMaster, m_rightDriveMaster);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("left Current", getCurrent()[0]);
    // SmartDashboard.putNumber("right Current", getCurrent()[1]);
  }

  public void arcadeDrive(double move, double turn) {
    m_differentialDrive.arcadeDrive(move, turn);
  }
  
  // /**
  //  * @return left current[0] and right current[1]
  //  */
  // public double[] getCurrent() {
  //   double[] outputCurrent = new double[2];
  //   outputCurrent[0] = m_leftDrive1.getStatorCurrent();
  //   outputCurrent[1] = m_rightDrive1.getStatorCurrent();
  //   return outputCurrent;
  // }

  /**
   * drives forward using motion magic
   * @param distance distance to go in inches
  */
  public void drive(final double distance) {
    m_leftDriveMaster.set(ControlMode.MotionMagic, inchesToEncoderTicks(distance));
    m_rightDriveMaster.set(ControlMode.MotionMagic, inchesToEncoderTicks(distance));
  }

  public double encoderTicksToInches(int number) {
    return (double)number / m_encoderResolution * m_gearRatio * m_wheelCircumference;
  }

  public int inchesToEncoderTicks(double inches) {
    return (int)(inches / m_wheelCircumference / m_gearRatio * m_encoderResolution);
  }
}
