package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpiutil.math.MathUtil;

public class PIDDrivetrain extends Drivetrain {
  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_leftPGainEntry;
  private final NetworkTableEntry m_leftIGainEntry;
  private final NetworkTableEntry m_leftDGainEntry;
  private final NetworkTableEntry m_rightPGainEntry;
  private final NetworkTableEntry m_rightIGainEntry;
  private final NetworkTableEntry m_rightDGainEntry;

  private final NetworkTableEntry m_leftFeedForwardEntry;
  private final NetworkTableEntry m_rightFeedForwardEntry;

  private final NetworkTableEntry m_leftOutputEntry;
  private final NetworkTableEntry m_rightOutputEntry;

  protected PIDController m_leftPIDcontroller;
  protected PIDController m_rightPIDcontroller;
  
  private double k_lP = 1.0; //left PID
  private double k_lI = 0.0; //TODO: tune these
  private double k_lD = 0.0;

  private double k_rP = 1.0; //right PID -- right P has to be negative i think
  private double k_rI = 0.0; 
  private double k_rD = 0.0;

  private final double k_staticGain = 1;  //TODO: thses porbable need to be tuned as well
  private final double k_velocityGain = 2;

  private final double m_trackWidth = 0.67; // Track width in meters

  private double leftFeedForeward;
  private double rightFeedForeward;
  private double rightOutput;
  private double leftOutput;
  private double leftSpeed;
  private double rightSpeed;

  private DifferentialDriveKinematics m_kinematics;
  private DifferentialDriveOdometry m_odometry;
  private SimpleMotorFeedforward m_feedforward;
  private DifferentialDriveWheelSpeeds wheelSpeeds;

  public final double k_maxSpeed = 3; //max speed in m/s
  public final double k_maxTurn = 2 * Math.PI; //max turn rate radians/s

  public PIDDrivetrain() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_leftPGainEntry = m_networkTable.getEntry("Left P gain");
    m_leftIGainEntry = m_networkTable.getEntry("Left I gain");
    m_leftDGainEntry = m_networkTable.getEntry("Left D gain");
    m_rightPGainEntry = m_networkTable.getEntry("Right P gain");
    m_rightIGainEntry = m_networkTable.getEntry("Right I gain");
    m_rightDGainEntry = m_networkTable.getEntry("Right D gain");

    m_leftFeedForwardEntry = m_networkTable.getEntry("Left feed forward");
    m_rightFeedForwardEntry = m_networkTable.getEntry("Right feed forward");
    
    m_leftOutputEntry = m_networkTable.getEntry("Left output");
    m_rightOutputEntry = m_networkTable.getEntry("Right output");

    m_rightPIDcontroller = new PIDController(k_rP, k_rI, k_rD, 0.2);
    m_leftPIDcontroller = new PIDController(k_lP, k_lI, k_lD, 0.2);

    m_kinematics = new DifferentialDriveKinematics(m_trackWidth);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));
    m_feedforward = new SimpleMotorFeedforward(k_staticGain, k_velocityGain);
    wheelSpeeds = new DifferentialDriveWheelSpeeds();
  }

  @Override
  public void periodic() {
    super.periodic(); 

    k_lP = m_leftPGainEntry.getDouble(k_lP);
    k_lI = m_leftIGainEntry.getDouble(k_lI);
    k_lD = m_leftDGainEntry.getDouble(k_lD);

    k_rP = m_rightPGainEntry.getDouble(k_rP);
    k_rI = m_rightIGainEntry.getDouble(k_rI);
    k_rD = m_rightDGainEntry.getDouble(k_rD);

    m_leftPIDcontroller.setPID(k_lP, k_lI, k_lD);
    m_rightPIDcontroller.setPID(k_rP, k_rI, k_rD);
    updateOdometry();

    m_leftPGainEntry.setNumber(k_lP);
    m_leftIGainEntry.setNumber(k_lI);
    m_leftDGainEntry.setNumber(k_lD);
    m_rightPGainEntry.setNumber(k_rP);
    m_rightIGainEntry.setNumber(k_rI);
    m_rightDGainEntry.setNumber(k_rD);

    m_leftFeedForwardEntry.setNumber(leftFeedForeward);
    m_rightFeedForwardEntry.setNumber(rightFeedForeward);
  }

  public void driveMotors(DifferentialDriveWheelSpeeds speeds) {
    leftFeedForeward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    rightFeedForeward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    leftOutput = m_leftPIDcontroller.calculate(getLeftEncoderRate(), speeds.leftMetersPerSecond);
    rightOutput = m_rightPIDcontroller.calculate(getRightEncoderRate(), speeds.rightMetersPerSecond);
    leftSpeed = leftOutput + leftFeedForeward;
    rightSpeed = rightOutput + rightFeedForeward;
    leftSpeed = MathUtil.clamp(leftSpeed, -12.0, 12.0);
    rightSpeed = MathUtil.clamp(rightSpeed, -12.0, 12.0);

    m_leftDriveMaster.setVoltage(-leftSpeed);
    m_rightDriveMaster.setVoltage(rightSpeed);
  }

  public void drive(double xSpeed, double rot) {
    wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    driveMotors(wheelSpeeds);
  }

  public void updateOdometry() {
    m_odometry.update(Rotation2d.fromDegrees(getYaw()), getLeftEncoderDistance(), getRightEncoderDistance());
  }
}