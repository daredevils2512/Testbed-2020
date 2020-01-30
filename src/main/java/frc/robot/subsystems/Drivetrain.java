/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final int m_leftDriveMasterID = 1;
  private final int m_leftDrive1ID = 2;
  private final int m_rightDriveMasterID = 4;
  private final int m_rightDrive1ID = 6;

  private final int m_leftEncoderChannelA = 2; 
  private final int m_leftEncoderChannelB = 3; 
  private final int m_rightEncoderChannelA = 0;
  private final int m_rightEncoderChannelB = 1; 

  protected final WPI_TalonSRX m_leftDriveMaster;
  protected final WPI_TalonSRX m_leftDrive1;
  protected final WPI_TalonSRX m_rightDriveMaster;
  protected final WPI_TalonSRX m_rightDrive1;

  protected final DifferentialDrive m_differentialDrive;

  protected Encoder m_leftEncoder;
  protected Encoder m_rightEncoder;

  protected double k_wheelDiameter = 4 * 0.02554; //inches time the conversion to meters
  protected double k_encoderResolution = 128; //provavly //also provabel not 4096

  private final int m_pigeonID = 0;
  private final PigeonIMU m_pigeon;
  private double[] ypr = {0.0, 0.0, 0.0};

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

    m_rightEncoder = new Encoder(m_rightEncoderChannelA, m_rightEncoderChannelB);
    m_leftEncoder = new Encoder(m_leftEncoderChannelA, m_leftEncoderChannelB);

    m_rightEncoder.setReverseDirection(true);

    // Distance in meters
    m_leftEncoder.setDistancePerPulse(Math.PI * k_wheelDiameter / k_encoderResolution);
    m_rightEncoder.setDistancePerPulse(Math.PI * k_wheelDiameter / k_encoderResolution);

    m_pigeon = new PigeonIMU(m_pigeonID);
    m_pigeon.configFactoryDefault();
    resetGyro();

    m_differentialDrive = new DifferentialDrive(m_leftDriveMaster, m_rightDriveMaster);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left ticks", getLeftEncoderTicks());
    SmartDashboard.putNumber("left inches", getLeftEncoderDistance());
    SmartDashboard.putNumber("left encoder speed", getLeftEncoderRate());
    SmartDashboard.putNumber("right ticks", getRightEncoderTicks());
    SmartDashboard.putNumber("right inches", getRightEncoderDistance());
    SmartDashboard.putNumber("right encoder speed", getRightEncoderRate());
    SmartDashboard.putNumber("yaw", getYaw());
    SmartDashboard.putNumber("pitch", getPitch());
    SmartDashboard.putNumber("roll", getRoll());
  }

  public void arcadeDrive(double move, double turn) {
    m_differentialDrive.arcadeDrive(move, turn);
  }

  public int getLeftEncoderTicks() {
    return m_leftEncoder.get();
  }

  public double getLeftEncoderDistance() {
    return m_leftEncoder.getDistance();
  }

  public double getLeftEncoderRate() {
    return m_leftEncoder.getRate();
  }

  public int getRightEncoderTicks() {
    return m_rightEncoder.get();
  }

  public double getRightEncoderDistance() {
    return m_rightEncoder.getDistance();
  }

  public double getRightEncoderRate() {
    return m_rightEncoder.getRate();
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double[] getYPR() {
    this.m_pigeon.getYawPitchRoll(ypr);
    return ypr;
  }

  public double getYaw() {
    return getYPR()[0];
  }

  public double getPitch() {
    return getYPR()[1];
  }

  public double getRoll() {
    return getYPR()[2];
  }

  public void resetGyro() {
    m_pigeon.setYaw(0.0);
    m_pigeon.setFusedHeading(0.0);
    m_pigeon.setAccumZAngle(0.0);
  }

  public boolean getHasResetGyro() {
    return m_pigeon.hasResetOccurred();
  }
}
