/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

/**
 * The drivetrain is a 6 wheel west coast differential drivetrain
 * with two-gear transmission. It consists of four {@link TalonFX}
 * motor controllers for driving (two per side), a {@link DoubleSolenoid}
 * for shifting, two 256PPR optical encoders (one per side)
 * mounted to the output of the gearbox for distance calculation,
 * and a {@link PigeonIMU} for heading calculation.
 */
public class Drivetrain2020 extends SubsystemBase implements KinematicsDrivetrain, OdometryDrivetrain {
  /**
   * All network table enties are stored as variables so they can be referenced
   * more reliably (instead of by name via string)
   */
  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_leftPGainEntry;
  private final NetworkTableEntry m_leftIGainEntry;
  private final NetworkTableEntry m_leftDGainEntry;
  private final NetworkTableEntry m_rightPGainEntry;
  private final NetworkTableEntry m_rightIGainEntry;
  private final NetworkTableEntry m_rightDGainEntry;
  private final NetworkTableEntry m_invertedDrivingEntry;
  private final NetworkTableEntry m_lowGearEntry;
  private final NetworkTableEntry m_yawEntry;
  private final NetworkTableEntry m_pitchEntry;
  private final NetworkTableEntry m_rollEntry;
  private final NetworkTableEntry m_headingEntry;

  private final int m_leftDriveMasterID = 10;
  private final int m_leftDriveFollowerID = 11;
  private final int m_rightDriveMasterID = 12;
  private final int m_rightDriveFollowerID = 13;

  private final WPI_TalonFX m_leftDriveMaster;
  private final WPI_TalonFX m_leftDriveFollower;
  private final WPI_TalonFX m_rightDriveMaster;
  private final WPI_TalonFX m_rightDriveFollower;

  private final int m_leftEncoderChannelA = 0;
  private final int m_leftEncoderChannelB = 1;
  private final int m_rightEncoderChannelA = 2;
  private final int m_rightEncoderChannelB = 3;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private final int m_pigeonID = 14;
  private final PigeonIMU m_pigeon;

  // private final int m_shifterForwardChannel = -1;
  // private final int m_shifterReverseChannel = -1;
  // private final DoubleSolenoid m_shifter;
  // private final DoubleSolenoid.Value m_highGearValue = Value.kForward;
  // private final DoubleSolenoid.Value m_lowGearValue = Value.kReverse;

  // Calculated using 7.16 meters = 25540 ticks;
  private final double m_distancePerTickApprox = 0.0002803; // Distance in meters
  private final int m_encoderResolution = 256;
  private final double m_gearRatio = (double)3 / 1;
  private final double m_wheelDiameter = Units.inchesToMeters(6); // Wheel diameter in meters
  private final double m_wheelCircumference = Math.PI * m_wheelDiameter; // Circumference in meters
  // TODO: Find out track width (can be calculated using the characterization tool)
  private final double m_trackWidth = Units.inchesToMeters(28);
  // TODO: Find out max speeds for low and high gear
  private final double m_maxSpeedHighGear = 3; // Max speed in high gear in meters per second
  private final double m_maxSpeedLowGear = 1; // Max speed in low gear in meters per second
  private final double m_maxAngularSpeedLowGear = 3;
  private final double m_maxAngularSpeedHighGear = 1;

  private final double m_staticGainHighGear = 1;
  private final double m_velocityGainHighGear = 8;
  private final double m_accelerationGainHighGear = 0;

  private final double m_staticGainLowGear = 1;
  private final double m_velocityGainLowGear = 8;
  private final double m_accelerationGainLowGear = 0;

  private boolean m_isDrivingInverted = false;

  private double[] m_gyroData = new double[3]; // Yaw, pitch, and roll in degrees

  private final DifferentialDriveKinematics m_kinematics;
  // TODO: Tune feedforward values using the characterization tool
  private final SimpleMotorFeedforward m_driveMotorFeedforwardHighGear;
  private final SimpleMotorFeedforward m_driveMotorFeedforwardLowGear;
  private final PIDController m_leftPIDController;
  private final PIDController m_rightPIDController;
  // TODO: Tune velocity PID
  private double m_leftPGain = 0;
  private double m_leftIGain = 0;
  private double m_leftDGain = 0;
  private double m_rightPGain = 0;
  private double m_rightIGain = 0;
  private double m_rightDGain = 0;

  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain2020() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_leftPGainEntry = m_networkTable.getEntry("Left P gain");
    m_leftIGainEntry = m_networkTable.getEntry("Left I gain");
    m_leftDGainEntry = m_networkTable.getEntry("Left D gain");
    m_rightPGainEntry = m_networkTable.getEntry("Right P gain");
    m_rightIGainEntry = m_networkTable.getEntry("Right I gain");
    m_rightDGainEntry = m_networkTable.getEntry("Right D gain");
    m_invertedDrivingEntry = m_networkTable.getEntry("Inverted driving");
    m_lowGearEntry = m_networkTable.getEntry("Low gear");
    m_yawEntry = m_networkTable.getEntry("Yaw");
    m_pitchEntry = m_networkTable.getEntry("Pitch");
    m_rollEntry = m_networkTable.getEntry("Roll");
    m_headingEntry = m_networkTable.getEntry("Heading");

    m_leftDriveMaster = new WPI_TalonFX(m_leftDriveMasterID);
    m_leftDriveFollower = new WPI_TalonFX(m_leftDriveFollowerID);
    m_rightDriveMaster = new WPI_TalonFX(m_rightDriveMasterID);
    m_rightDriveFollower = new WPI_TalonFX(m_rightDriveFollowerID);

    // Config to factory defaults to prevent unexpected behavior
    m_leftDriveMaster.configFactoryDefault();
    m_leftDriveFollower.configFactoryDefault();
    m_rightDriveMaster.configFactoryDefault();
    m_rightDriveFollower.configFactoryDefault();

    // Designate drive masters
    m_leftDriveFollower.follow(m_leftDriveMaster);
    m_rightDriveFollower.follow(m_rightDriveMaster);

    m_leftDriveMaster.setInverted(InvertType.InvertMotorOutput);
    m_leftDriveFollower.setInverted(InvertType.FollowMaster);
    m_rightDriveMaster.setInverted(InvertType.None);
    m_rightDriveFollower.setInverted(InvertType.FollowMaster);

    m_leftEncoder = new Encoder(m_leftEncoderChannelA, m_leftEncoderChannelB);
    m_rightEncoder = new Encoder(m_rightEncoderChannelA, m_rightEncoderChannelB);
    // m_leftEncoder.setDistancePerPulse((double)1 / m_encoderResolution * m_gearRatio * m_wheelCircumference);
    // m_rightEncoder.setDistancePerPulse((double)1 / m_encoderResolution * m_gearRatio * m_wheelCircumference);
    m_leftEncoder.setDistancePerPulse(m_distancePerTickApprox);
    m_rightEncoder.setDistancePerPulse(m_distancePerTickApprox);
    m_leftEncoder.setReverseDirection(false);
    m_rightEncoder.setReverseDirection(true);

    m_pigeon = new PigeonIMU(m_pigeonID);
    m_pigeon.configFactoryDefault();
    m_pigeon.setFusedHeading(0);

    // m_shifter = new DoubleSolenoid(m_shifterForwardChannel, m_shifterReverseChannel);

    m_driveMotorFeedforwardHighGear = new SimpleMotorFeedforward(m_staticGainHighGear, m_velocityGainHighGear, m_accelerationGainHighGear);
    m_driveMotorFeedforwardLowGear = new SimpleMotorFeedforward(m_staticGainLowGear, m_velocityGainLowGear, m_accelerationGainLowGear);
    m_kinematics = new DifferentialDriveKinematics(m_trackWidth);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));
    m_leftPIDController = new PIDController(m_leftPGain, m_leftIGain, m_leftDGain);
    m_rightPIDController = new PIDController(m_rightPGain, m_rightIGain, m_rightDGain);
  }

  @Override
  public void periodic() {
    m_leftPGain = m_leftPGainEntry.getNumber(m_leftPGain).doubleValue();
    m_leftIGain = m_leftIGainEntry.getNumber(m_leftIGain).doubleValue();
    m_leftDGain = m_leftDGainEntry.getNumber(m_leftDGain).doubleValue();
    m_rightPGain = m_rightPGainEntry.getNumber(m_rightPGain).doubleValue();
    m_rightIGain = m_rightIGainEntry.getNumber(m_rightIGain).doubleValue();
    m_rightDGain = m_rightDGainEntry.getNumber(m_rightDGain).doubleValue();

    m_leftPIDController.setPID(m_leftPGain, m_leftIGain, m_leftDGain);
    m_rightPIDController.setPID(m_rightPGain, m_rightIGain, m_rightDGain);

    updateGyroData();
    updateOdometry();

    m_leftPGainEntry.setNumber(m_leftPGain);
    m_leftIGainEntry.setNumber(m_leftIGain);
    m_leftDGainEntry.setNumber(m_leftDGain);
    m_rightPGainEntry.setNumber(m_leftPGain);
    m_rightIGainEntry.setNumber(m_leftIGain);
    m_rightDGainEntry.setNumber(m_leftDGain);

    m_invertedDrivingEntry.setBoolean(getDrivingInverted());
    m_lowGearEntry.setBoolean(getLowGear());
    m_yawEntry.setNumber(getYaw());
    m_pitchEntry.setNumber(getPitch());
    m_rollEntry.setNumber(getRoll());
    m_headingEntry.setNumber(getHeading());
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
  public void resetDriveEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  } 
  
  /**
   * Get the current max speed depending on the current gear
   * @return Speed in meters per second
   */
  public double getMaxSpeed() {
    return getLowGear() ? m_maxSpeedLowGear : m_maxSpeedHighGear;
  }

  @Override
  public double getMaxAngularSpeed() {
    return getLowGear() ? m_maxAngularSpeedLowGear : m_maxAngularSpeedHighGear;
  }

  public boolean getLowGear() {
    // return m_shifter.get() == m_lowGearValue;
    return false;
  }

  public void setLowGear(final boolean wantsLowGear) {
    // m_shifter.set(wantsLowGear ? m_lowGearValue : m_highGearValue);
  }

  public void setDrivingInverted(boolean wantsInverted) {
    m_isDrivingInverted = wantsInverted;
  }

  public boolean getDrivingInverted() {
    return m_isDrivingInverted;
  }
  
  @Override
  public void arcadeDrive(double move, double turn) {
    m_leftDriveMaster.set(ControlMode.PercentOutput, move - turn);
    m_rightDriveMaster.set(ControlMode.PercentOutput, move + turn);
  }

  /**
   * Set the drivetrain's linear and angular target velocities
   * @param velocity Velocity in meters per second
   * @param angularVelocity Angular velocity in radians per second
   */
  @Override
  public void velocityArcadeDrive(double velocity, double angularVelocity) {
    velocity = m_isDrivingInverted ? -velocity : velocity;
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(velocity, 0, angularVelocity)));
  }

  private void setSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    double leftFeedforward;
    double rightFeedforward;

    if (getLowGear()) {
      leftFeedforward = m_driveMotorFeedforwardLowGear.calculate(wheelSpeeds.leftMetersPerSecond);
      rightFeedforward = m_driveMotorFeedforwardLowGear.calculate(wheelSpeeds.rightMetersPerSecond);
    } else {
      leftFeedforward = m_driveMotorFeedforwardHighGear.calculate(wheelSpeeds.leftMetersPerSecond);
      rightFeedforward = m_driveMotorFeedforwardHighGear.calculate(wheelSpeeds.rightMetersPerSecond);
    }

    double leftPIDOutput = m_leftPIDController.calculate(getLeftVelocity(), wheelSpeeds.leftMetersPerSecond);
    double rightPIDOutput = m_rightPIDController.calculate(getRightVelocity(), wheelSpeeds.rightMetersPerSecond);

    m_leftDriveMaster.set(leftFeedforward + leftPIDOutput);
    m_rightDriveMaster.set(rightFeedforward + rightPIDOutput);
  }

  private double getYaw() {
    return m_gyroData[0];
  }

  private double getPitch() {
    return m_gyroData[1];
  }

  private double getRoll() {
    return m_gyroData[2];
  }

  @Override
  public double getHeading() {
    return m_pigeon.getFusedHeading();
  }

  @Override
  public void resetHeading() {
    resetHeading();
  }

  /**
   * Set a new heading for the drivetrain
   * @param angle
   */
  public void setHeading(double angle) {
    m_pigeon.setFusedHeading(angle);
  }
  
  /**
   * Must be called periodically to retrieve gyro data from the {@link PigeonIMU}
   */
  private void updateGyroData() {
    m_pigeon.getYawPitchRoll(m_gyroData);
  }


  /**
   * Get the current pose (rotation and translation) of the robot
   * @return Pose with translation in meters
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public void resetPose() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_pigeon.setFusedHeading(0);
    Pose2d newPose = new Pose2d();
    m_odometry.resetPosition(newPose, Rotation2d.fromDegrees(m_pigeon.getFusedHeading()));
  }

  @Override
  public void resetPose(Pose2d newPose) {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_pigeon.setFusedHeading(0);
    m_odometry.resetPosition(newPose, Rotation2d.fromDegrees(m_pigeon.getFusedHeading()));
  }

  /**
   * Must be called periodically to maintain an accurate position and heading
   */
  private void updateOdometry() {
    m_odometry.update(Rotation2d.fromDegrees(getYaw()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }
}
