/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class AleaDrivetrain extends SubsystemBase implements SimpleDrivetrain {
  private final int m_leftDriveMasterID = 10;
  private final int m_leftDriveFollowerID = 11;
  private final int m_rightDriveMasterID = 12;
  private final int m_rightDriveFollowerID = 13;

  private final WPI_TalonSRX m_leftDriveMaster;
  private final WPI_TalonSRX m_leftDriveFollower;
  private final WPI_TalonSRX m_rightDriveMaster;
  private final WPI_TalonSRX m_rightDriveFollower;

  private final int m_leftEncoderChannelA = 6;
  private final int m_leftEncoderChannelB = 7;
  private final int m_rightEncoderChannelA = 4;
  private final int m_rightEncoderChannelB = 5;
  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private final double m_wheelDiameter = Units.inchesToMeters(4); // Diameter in meters
  private final int m_encoderResolution = 128; // Pulses per revolution

  /**
   * Creates a new AtlasDrivetrain.
   */
  public AleaDrivetrain() {
    m_leftDriveMaster = new WPI_TalonSRX(m_leftDriveMasterID);
    m_leftDriveFollower = new WPI_TalonSRX(m_leftDriveFollowerID);
    m_rightDriveMaster = new WPI_TalonSRX(m_rightDriveMasterID);
    m_rightDriveFollower = new WPI_TalonSRX(m_rightDriveFollowerID);
    m_leftDriveMaster.configFactoryDefault();
    m_leftDriveFollower.configFactoryDefault();
    m_rightDriveMaster.configFactoryDefault();
    m_rightDriveFollower.configFactoryDefault();
    
    m_leftDriveMaster.setInverted(InvertType.InvertMotorOutput);
    m_leftDriveFollower.setInverted(InvertType.FollowMaster);
    m_rightDriveMaster.setInverted(InvertType.None);
    m_rightDriveFollower.setInverted(InvertType.FollowMaster);

    m_leftDriveFollower.follow(m_leftDriveMaster);
    m_rightDriveFollower.follow(m_rightDriveMaster);

    m_leftEncoder = new Encoder(m_leftEncoderChannelA, m_leftEncoderChannelB);
    m_rightEncoder = new Encoder(m_rightEncoderChannelA, m_rightEncoderChannelB);
    m_leftEncoder.setDistancePerPulse(Math.PI * m_wheelDiameter / m_encoderResolution);
    m_rightEncoder.setDistancePerPulse(Math.PI * m_wheelDiameter / m_encoderResolution);
    m_leftEncoder.setReverseDirection(false);
    m_rightEncoder.setReverseDirection(true);
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void arcadeDrive(double move, double turn) {
    m_leftDriveMaster.set(ControlMode.PercentOutput, move + turn);
    m_rightDriveMaster.set(ControlMode.PercentOutput, move - turn);
  }
}

