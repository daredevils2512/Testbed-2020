/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Properties;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
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

public final class AtlasDrivetrain extends SubsystemBase implements KinematicsDrivetrain, OdometryDrivetrain {
  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_leftVelocityEntry;
  private final NetworkTableEntry m_rightVelocityEntry;
  private final NetworkTableEntry m_xPositionEntry;
  private final NetworkTableEntry m_yPositionEntry;
  private final NetworkTableEntry m_headingEntry;

  private final int m_leftDriveMasterID;
  private final int m_leftDriveFollowerID;
  private final int m_rightDriveMasterID;
  private final int m_rightDriveFollowerID;
  private final WPI_TalonSRX m_leftDriveMaster;
  private final WPI_TalonSRX m_leftDriveFollower;
  private final WPI_TalonSRX m_rightDriveMaster;
  private final WPI_TalonSRX m_rightDriveFollower;

  private final int m_leftEncoderChannelA = 0;
  private final int m_leftEncoderChannelB = 1;
  private final int m_rightEncoderChannelA = 2;
  private final int m_rightEncoderChannelB = 3;
  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private final int m_pigeonID = 3;
  private final PigeonIMU m_pigeon;

  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDriveOdometry m_odometry;
  private final SimpleMotorFeedforward m_feedforward;
  private final PIDController m_leftPIDController;
  private final PIDController m_rightPIDController;
  private Properties properties;

  private static final String PROPERTIES_FILE = "/atlas.properties";


  private double m_leftPGain = 0;
  private double m_leftIGain = 0;
  private double m_leftDGain = 0;
  private double m_rightPGain = 0;
  private double m_rightIGain = 0;
  private double m_rightDGain = 0;

  private final double m_maxSpeed = 1; // Speed in meters per second
  private final double m_maxAngularSpeed = 4; // Angular speed in radians per second

  private final double m_distancePerTickApprox = 0.0006; // Distance in meters
  private final double m_wheelDiameter = Units.inchesToMeters(4); // Diameter in meters
  private final double m_gearRatio = 1; // Nobody knows what this is
  private final double m_trackWidth = 0.67; // Width in meters
  private final double m_staticGain = 0.35; // Guess based on Alea tuning
  private final double m_velocityGain = 9; // Guess based on Alea tuning
  private final int m_encoderResolution = 128; // Ticks per revolution

  private double[] m_gyroData = new double[3]; // Yaw, pitch, roll set by the pigeon

  /**
   * Creates a new AtlasDrivetrain.
   */
  public AtlasDrivetrain() {  

    Properties defaultProperties = new Properties();
    properties = new Properties(defaultProperties);
    try {
      InputStream inputStream = new FileInputStream(Filesystem.getDeployDirectory() + PROPERTIES_FILE);
      defaultProperties.load(inputStream);
      InputStream robotStream = new FileInputStream(Filesystem.getOperatingDirectory() + PROPERTIES_FILE);
      properties.load(robotStream);
    } catch(IOException e) {
      e.printStackTrace();
    }    

    m_leftDriveMasterID = Integer.parseInt(properties.getProperty("drivetrain.leftMasterId"));
    m_leftDriveFollowerID = Integer.parseInt(properties.getProperty("drivetrain.leftFollowerId"));
    m_rightDriveMasterID = Integer.parseInt(properties.getProperty("drivetrain.rightMasterId"));
    m_rightDriveFollowerID = Integer.parseInt(properties.getProperty("drivetrain.rightFollowerId"));

    m_leftPGain = Double.parseDouble(properties.getProperty("drivetrain.leftPGain", "0.0"));
    m_leftIGain = Double.parseDouble(properties.getProperty("drivetrain.leftIGain", "0.0"));
    m_leftDGain = Double.parseDouble(properties.getProperty("drivetrain.leftDGain", "0.0"));

    m_rightPGain = Double.parseDouble(properties.getProperty("drivetrain.rightPGain", "0.0"));
    m_rightIGain = Double.parseDouble(properties.getProperty("drivetrain.rightIGain", "0.0"));
    m_rightDGain = Double.parseDouble(properties.getProperty("drivetrain.rightDGain", "0.0"));

    m_leftDriveMaster = new WPI_TalonSRX(m_leftDriveMasterID);
    m_leftDriveFollower = new WPI_TalonSRX(m_leftDriveFollowerID);
    m_rightDriveMaster = new WPI_TalonSRX(m_rightDriveMasterID);
    m_rightDriveFollower = new WPI_TalonSRX(m_rightDriveFollowerID);
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_leftVelocityEntry = m_networkTable.getEntry("Left velocity");
    m_rightVelocityEntry = m_networkTable.getEntry("Right velocity");
    m_xPositionEntry = m_networkTable.getEntry("X position");
    m_yPositionEntry = m_networkTable.getEntry("Y position");
    m_headingEntry = m_networkTable.getEntry("Heading");


    m_leftDriveMaster.configFactoryDefault();
    m_leftDriveFollower.configFactoryDefault();
    m_rightDriveMaster.configFactoryDefault();
    m_rightDriveFollower.configFactoryDefault();

    m_leftDriveFollower.follow(m_leftDriveMaster);
    m_rightDriveFollower.follow(m_rightDriveMaster);

    m_leftDriveMaster.setInverted(InvertType.InvertMotorOutput);
    m_leftDriveFollower.setInverted(InvertType.FollowMaster);
    m_rightDriveMaster.setInverted(InvertType.None);
    m_rightDriveFollower.setInverted(InvertType.FollowMaster);

    m_leftEncoder = new Encoder(m_leftEncoderChannelA, m_leftEncoderChannelB);
    m_rightEncoder = new Encoder(m_rightEncoderChannelA, m_rightEncoderChannelB);
    // m_leftEncoder.setDistancePerPulse(1.0 / m_encoderResolution * m_gearRatio * m_wheelDiameter * Math.PI);
    // m_rightEncoder.setDistancePerPulse(1.0 / m_encoderResolution * m_gearRatio * m_wheelDiameter * Math.PI);
    m_leftEncoder.setDistancePerPulse(m_distancePerTickApprox);
    m_rightEncoder.setDistancePerPulse(m_distancePerTickApprox);
    m_leftEncoder.setReverseDirection(true);
    m_rightEncoder.setReverseDirection(false);

    m_pigeon = new PigeonIMU(m_rightDriveMaster);
    m_pigeon.configFactoryDefault();

    m_kinematics = new DifferentialDriveKinematics(m_trackWidth);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    m_feedforward = new SimpleMotorFeedforward(m_staticGain, m_velocityGain);
    m_leftPIDController = new PIDController(m_leftPGain, m_leftIGain, m_leftDGain);
    m_rightPIDController = new PIDController(m_rightPGain, m_rightIGain, m_rightDGain);

    m_networkTable.getEntry("left P").setDouble(m_leftPGain); 
    m_networkTable.getEntry("left I").setDouble(m_leftIGain);
    m_networkTable.getEntry("left D").setDouble(m_leftDGain); 

    m_networkTable.getEntry("right P").setDouble(m_rightPGain); 
    m_networkTable.getEntry("right P").setDouble(m_rightIGain); 
    m_networkTable.getEntry("right P").setDouble(m_rightDGain); 

  }

  @Override
  public void periodic() {
    updateGyroData();
    updatePose();

    m_leftVelocityEntry.setNumber(getLeftVelocity());
    m_rightVelocityEntry.setNumber(getRightVelocity());
    m_xPositionEntry.setNumber(getPose().getTranslation().getX());
    m_yPositionEntry.setNumber(getPose().getTranslation().getY());
    m_headingEntry.setNumber(getPose().getRotation().getDegrees());
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
    return -m_pigeon.getFusedHeading();
  }

  @Override
  public double getMaxSpeed() {
    return m_maxSpeed;
  }

  @Override
  public double getMaxAngularSpeed() {
    return m_maxAngularSpeed;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public void resetDriveEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    Pose2d newPose = new Pose2d(0, 0, Rotation2d.fromDegrees(getHeading()));
    m_odometry.resetPosition(newPose, Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void resetHeading() {
    m_pigeon.setFusedHeading(0);
    m_pigeon.setYaw(0);
    Pose2d newPose = new Pose2d(m_odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0));
    m_odometry.resetPosition(newPose, Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void arcadeDrive(double move, double turn) {
    m_leftDriveMaster.set(ControlMode.PercentOutput, move + turn);
    m_rightDriveMaster.set(ControlMode.PercentOutput, move - turn);
  }

  /**
   * Set the drivetrain's linear and angular velocity targets
   * @param velocity Linear velocity in meters per second
   * @param angularVelocity Angular velocity in radians per second
   */
  @Override
  public void velocityArcadeDrive(double velocity, double angularVelocity) {
    velocity = MathUtil.clamp(velocity, -m_maxSpeed, m_maxSpeed);
    angularVelocity = MathUtil.clamp(angularVelocity, -m_maxAngularSpeed, m_maxAngularSpeed);
    setWheelSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(velocity, 0, angularVelocity)));
  }

  public void voltageTank(double left, double right) {
    m_leftDriveMaster.setVoltage(left);
    m_rightDriveMaster.setVoltage(right);
  }

  private void setWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    m_leftPGain = m_networkTable.getEntry("left P").getDouble(0.0);
    m_leftIGain = m_networkTable.getEntry("left I").getDouble(0.0);
    m_leftDGain = m_networkTable.getEntry("left D").getDouble(0.0);

    m_rightPGain = m_networkTable.getEntry("right P").getDouble(0.0);
    m_rightIGain = m_networkTable.getEntry("right I").getDouble(0.0);
    m_rightDGain = m_networkTable.getEntry("right D").getDouble(0.0);

    double leftFeedforward = m_feedforward.calculate(wheelSpeeds.leftMetersPerSecond);
    double rightFeedforward = m_feedforward.calculate(wheelSpeeds.rightMetersPerSecond);
    double leftPIDOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), wheelSpeeds.leftMetersPerSecond);
    double rightPIDOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), wheelSpeeds.rightMetersPerSecond);

    m_leftDriveMaster.setVoltage(leftFeedforward + leftPIDOutput);
    m_rightDriveMaster.setVoltage(rightFeedforward + rightPIDOutput);
  }

  @Override
  public void resetPose() {
    resetDriveEncoders();
    resetHeading();
    m_odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void resetPose(Pose2d newPose) {
    resetDriveEncoders();
    resetHeading();
    m_odometry.resetPosition(newPose, Rotation2d.fromDegrees(getHeading()));
  }

  private void updateGyroData() {
    m_pigeon.getYawPitchRoll(m_gyroData);
  }

  private void updatePose() {
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance());
  }

  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public void savePID() {
    try {
      OutputStream outputStream = new FileOutputStream(Filesystem.getOperatingDirectory() + PROPERTIES_FILE);
      properties.setProperty("leftPGain", "" + m_networkTable.getEntry("left P").getDouble(0.0));
      properties.setProperty("leftIGain", "" + m_networkTable.getEntry("left I").getDouble(0.0));
      properties.setProperty("leftDGain", "" + m_networkTable.getEntry("left D").getDouble(0.0));

      properties.setProperty("rightPGain", "" + m_networkTable.getEntry("right P").getDouble(0.0));
      properties.setProperty("rightIGain", "" + m_networkTable.getEntry("right I").getDouble(0.0));
      properties.setProperty("rightDGain", "" + m_networkTable.getEntry("right D").getDouble(0.0));
      properties.store(outputStream, "saved pid");

    } catch(IOException e) {
      e.printStackTrace();
    }
  }
}
