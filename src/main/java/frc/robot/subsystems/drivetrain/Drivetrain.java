/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final int m_leftDriveMasterID = 1;
  private final int m_leftDrive1ID = 2;
  private final int m_rightDriveMasterID = 3;
  private final int m_rightDrive1ID = 4;

  private final int m_leftEncoderChannelA = 0; 
  private final int m_leftEncoderChannelB = 1; 
  private final int m_rightEncoderChannelA = 2;
  private final int m_rightEncoderChannelB = 3; 

  protected final WPI_TalonSRX m_leftDriveMaster;
  protected final WPI_TalonSRX m_leftDrive1;
  protected final WPI_TalonSRX m_rightDriveMaster;
  protected final WPI_TalonSRX m_rightDrive1;

  protected final Encoder m_leftEncoder;
  protected final Encoder m_rightEncoder;

  protected double k_wheelDiameter = 4 * 0.0254; //inches time the conversion to meters
  protected double k_encoderResolution = 128; //provavly //also provabel not 4096

  private final int m_pigeonID = 5;
  protected final PigeonIMU m_pigeon;

  private double[] m_gyroData = new double[3]; // Yaw pitch roll in degrees

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
    //0.0236065636 for inches idk seems pretty accurate to me
    // m_leftEncoder.setDistancePerPulse((Math.PI * k_wheelDiameter) / k_encoderResolution);
    // m_rightEncoder.setDistancePerPulse((Math.PI * k_wheelDiameter) / k_encoderResolution);
    m_leftEncoder.setDistancePerPulse(0.0006); //the magic number iin meters
    m_rightEncoder.setDistancePerPulse(0.0006);

    m_pigeon = new PigeonIMU(m_pigeonID);
    m_pigeon.configFactoryDefault();

    resetEncoders();
    setHeading(0);
  }

  @Override
  public void periodic() {
    updateGyroData();

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

  /**
   * Must be called periodically to retrieve gyro data from the pigeon
   */
  public void updateGyroData() {
    m_pigeon.getYawPitchRoll(m_gyroData);
  }

  public double getYaw() {
    return m_gyroData[0];
  }

  public double getPitch() {
    return m_gyroData[1];
  }

  public double getRoll() {
    return m_gyroData[2];
  }

  /**
   * Set a new robot heading
   * @param angle
   */
  public void setHeading(double angle) {
    m_pigeon.setFusedHeading(angle);
  }
}
