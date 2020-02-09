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
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

public final class AleaDrivetrain extends SubsystemBase implements KinematicsDrivetrain, OdometryDrivetrain {
  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_leftDistanceEntry;
  private final NetworkTableEntry m_rightDistanceEntry;
  private final NetworkTableEntry m_leftVelocityEntry;
  private final NetworkTableEntry m_rightVelocityEntry;
  private final NetworkTableEntry m_leftVelocityErrorEntry;
  private final NetworkTableEntry m_rightVelocityErrorEntry;
  private final NetworkTableEntry m_yawEntry;
  private final NetworkTableEntry m_pitchEntry;
  private final NetworkTableEntry m_rollEntry;
  private final NetworkTableEntry m_xPositionEntry;
  private final NetworkTableEntry m_yPositionEntry;
  private final NetworkTableEntry m_headingEntry;

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

  private final double m_distancePerTickApprox = 0.0006; // Distance in meters
  private final double m_wheelDiameter = Units.inchesToMeters(4); // Diameter in meters
  private final double m_wheelCircumference = Math.PI * m_wheelDiameter;
  private final double m_gearRatio = (double)1 / 8; // Gearing between encoders and wheels
  private final double m_trackWidth = 0.67; // Width in meters
  private final int m_encoderResolution = 128; // Pulses per revolution
  private final double m_staticGain = 0.353; // Tuned on elevated drivetrain
  private final double m_velocityGain = 9.02; // Tuned on elevated drivetrain
  private final double m_accelerationGain = -0.0943; // Tuned on elevated drivetrain

  private final AHRS m_ahrs;

  private final DifferentialDriveKinematics m_kinematics;
  private final SimpleMotorFeedforward m_feedforward;
  private final PIDController m_leftVelocityController;
  private final PIDController m_rightVelocityController;
  
  private final DifferentialDriveOdometry m_odometry;

  private final double m_leftPGain = 0;
  private final double m_leftIGain = 0;
  private final double m_leftDGain = 0;
  private final double m_rightPGain = 0;
  private final double m_rightIGain = 0;
  private final double m_rightDGain = 0;

  private final double m_maxSpeed = 3; // Speed in meters per second
  private final double m_maxAngularSpeed = 4; // Angular speed in radians per second

  /**
   * Creates a new AtlasDrivetrain.
   */
  public AleaDrivetrain() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_leftDistanceEntry = m_networkTable.getEntry("Left distance");
    m_rightDistanceEntry = m_networkTable.getEntry("Right distance");
    m_leftVelocityEntry = m_networkTable.getEntry("Left velocity");
    m_rightVelocityEntry = m_networkTable.getEntry("Right velocity");
    m_leftVelocityErrorEntry = m_networkTable.getEntry("Left velocity error");
    m_rightVelocityErrorEntry = m_networkTable.getEntry("Right velocity error");
    m_yawEntry = m_networkTable.getEntry("Yaw");
    m_pitchEntry = m_networkTable.getEntry("Pitch");
    m_rollEntry = m_networkTable.getEntry("Roll");
    m_xPositionEntry = m_networkTable.getEntry("X position");
    m_yPositionEntry = m_networkTable.getEntry("Y position");
    m_headingEntry = m_networkTable.getEntry("Heading");

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
    // m_leftEncoder.setDistancePerPulse(1.0 / m_encoderResolution * m_gearRatio * m_wheelCircumference);
    // m_rightEncoder.setDistancePerPulse(1.0 / m_encoderResolution * m_gearRatio * m_wheelCircumference);
    m_leftEncoder.setDistancePerPulse(m_distancePerTickApprox);
    m_rightEncoder.setDistancePerPulse(m_distancePerTickApprox);
    m_leftEncoder.setReverseDirection(true);
    m_rightEncoder.setReverseDirection(false);

    m_ahrs = new AHRS();

    m_kinematics = new DifferentialDriveKinematics(m_trackWidth);
    m_feedforward = new SimpleMotorFeedforward(m_staticGain, m_velocityGain, m_accelerationGain);
    m_leftVelocityController = new PIDController(m_leftPGain, m_leftIGain, m_leftDGain);
    m_rightVelocityController = new PIDController(m_rightPGain, m_rightIGain, m_rightDGain);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    m_leftDistanceEntry.setNumber(getLeftDistance());
    m_rightDistanceEntry.setNumber(getRightDistance());
    m_leftVelocityEntry.setNumber(getLeftVelocity());
    m_rightVelocityEntry.setNumber(getRightVelocity());
    m_yawEntry.setNumber(m_ahrs.getYaw());
    m_pitchEntry.setNumber(m_ahrs.getPitch());
    m_rollEntry.setNumber(m_ahrs.getRoll());
    m_xPositionEntry.setNumber(getPose().getTranslation().getX());
    m_yPositionEntry.setNumber(getPose().getTranslation().getY());
    m_headingEntry.setNumber(getPose().getRotation().getDegrees());
  }
  
  @Override
  public double getMaxSpeed() {
    return m_maxSpeed;
  }

  @Override
  public double getMaxAngularSpeed() {
    return m_maxAngularSpeed;
  }
  @Override
  public double getLeftDistance() {
    return m_leftEncoder.getDistance();
  }

  @Override
  public double getRightDistance() {
    return m_rightEncoder.getDistance();
  }

  @Override
  public double getLeftVelocity() {
    return m_leftEncoder.getRate();
  }

  @Override
  public double getRightVelocity() {
    return m_rightEncoder.getRate();
  }

  @Override
  public double getHeading() {
    return getPose().getRotation().getDegrees();
  }

  @Override
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public void resetDriveEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    Pose2d newPose = new Pose2d(0, 0, m_odometry.getPoseMeters().getRotation());
    m_odometry.resetPosition(newPose, Rotation2d.fromDegrees(m_ahrs.getYaw()));
  }

  @Override
  public void resetHeading() {
    m_ahrs.reset();
    Pose2d newPose = new Pose2d(m_odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0));
    m_odometry.resetPosition(newPose, Rotation2d.fromDegrees(m_ahrs.getYaw()));
  }

  @Override
  public void resetPose() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_ahrs.reset();
    m_odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void arcadeDrive(double move, double turn) {
    m_leftDriveMaster.set(ControlMode.PercentOutput, move + turn);
    m_rightDriveMaster.set(ControlMode.PercentOutput, move - turn);
  }

  @Override
  public void velocityArcadeDrive(double velocity, double angularVelocity) {
    velocity = MathUtil.clamp(velocity, -m_maxSpeed, m_maxSpeed);
    angularVelocity = MathUtil.clamp(angularVelocity, -m_maxAngularSpeed, m_maxAngularSpeed);
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(velocity, 0, angularVelocity)));
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    double leftFeedforward = m_feedforward.calculate(wheelSpeeds.leftMetersPerSecond);
    double rightFeedforward = m_feedforward.calculate(wheelSpeeds.rightMetersPerSecond);
    double leftPIDOutput = m_leftVelocityController.calculate(getLeftVelocity(), wheelSpeeds.leftMetersPerSecond);
    double rightPIDOutput = m_rightVelocityController.calculate(getRightVelocity(), wheelSpeeds.rightMetersPerSecond);

    m_leftDriveMaster.setVoltage(leftFeedforward + leftPIDOutput);
    m_rightDriveMaster.setVoltage(rightFeedforward + rightPIDOutput);

    m_leftVelocityErrorEntry.setNumber(wheelSpeeds.leftMetersPerSecond);
    m_rightVelocityErrorEntry.setNumber(wheelSpeeds.rightMetersPerSecond);
  }

  public void updateOdometry() {
    m_odometry.update(Rotation2d.fromDegrees(m_ahrs.getYaw()), getLeftDistance(), getRightDistance());
  }
}
