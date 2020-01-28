package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

public class PIDdrivetrain extends Drivetrain {
  protected PIDController m_leftPIDcontroller;
  protected PIDController m_rightPIDcontroller;
  
  private double k_lP = 1.0; //left PID
  private double k_lI = 0.0; //TODO: tune these
  private double k_lD = 0.0;

  private double k_rP = -1.0; //right PID -- right P has to be negative i think
  private double k_rI = 0.0; 
  private double k_rD = 0.0;

  private final double m_trackWidth = 0.67;
  private final double k_staticGain = 1;
  private final double k_velocityGain = 2;
  public final double k_maxSpeed = 3; //max speed in m/s
  public final double k_maxTurn = Math.PI; //max turn rate radians/s

  private double leftFeedForeward;
  private double rightFeedForeward;
  private double rightOutput;
  private double leftOutput;

  private DifferentialDriveKinematics m_kinematics;
  private DifferentialDriveOdometry m_odometry;
  private SimpleMotorFeedforward m_feedforward;
  private DifferentialDriveWheelSpeeds wheelSpeeds;

  public PIDdrivetrain() {
    
    m_rightPIDcontroller = new PIDController(k_rP, k_rI, k_rD, 0.2);
    m_leftPIDcontroller = new PIDController(k_lP, k_lI, k_lD, 0.2);

    SmartDashboard.getEntry("left drivetrain P").setNumber(k_lP);
    SmartDashboard.getEntry("left drivetrain I").setNumber(k_lI);
    SmartDashboard.getEntry("left drivetrain D").setNumber(k_lD);
    SmartDashboard.getEntry("right drivetrian P").setNumber(k_rP);
    SmartDashboard.getEntry("right drivetrian I").setNumber(k_rI);
    SmartDashboard.getEntry("right drivetrian D").setNumber(k_rD);

    resetGyro();
    resetEncoders();

    m_kinematics = new DifferentialDriveKinematics(m_trackWidth);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()));
    m_feedforward = new SimpleMotorFeedforward(k_staticGain, k_velocityGain);
    wheelSpeeds = new DifferentialDriveWheelSpeeds();
  }

  @Override
  public void periodic() {
    super.periodic(); 
    k_lP = SmartDashboard.getEntry("left drivetrain P").getDouble(0.0);
    k_lI = SmartDashboard.getEntry("left drivetrain I").getDouble(0.0);
    k_lD = SmartDashboard.getEntry("left drivetrain D").getDouble(0.0);
    k_rP = SmartDashboard.getEntry("right drivetrain P").getDouble(0.0);
    k_rI = SmartDashboard.getEntry("right drivetrain I").getDouble(0.0);
    k_rD = SmartDashboard.getEntry("right drivetrain D").getDouble(0.0);
    updateOdometry();
    SmartDashboard.putNumber("left feed forewweard", leftFeedForeward);
    SmartDashboard.putNumber("right feed foreward", rightFeedForeward);
    SmartDashboard.putNumber("left output", leftOutput);
    SmartDashboard.putNumber("right output", rightOutput);
  }

  public void driveMotors(DifferentialDriveWheelSpeeds speeds) {
    // SmartDashboard.putNumber("left speed", speeds.leftMetersPerSecond);
    // SmartDashboard.putNumber("right speed", speeds.rightMetersPerSecond);
    leftFeedForeward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    rightFeedForeward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    leftOutput = m_leftPIDcontroller.calculate(getLeftEncoderRate(), speeds.leftMetersPerSecond);
    rightOutput = m_rightPIDcontroller.calculate(getRightEncoderRate(), speeds.rightMetersPerSecond);
    double leftSpeed = leftOutput + leftFeedForeward;
    double rightSpeed = rightOutput + rightFeedForeward;
    SmartDashboard.putNumber("left speed", leftSpeed);
    SmartDashboard.putNumber("right speed", rightSpeed);
    leftSpeed = MathUtil.clamp(leftSpeed, -12.0, 12.0);
    rightSpeed = MathUtil.clamp(rightSpeed, -12.0, 12.0);
    m_leftDriveMaster.setVoltage(leftSpeed);
    m_rightDriveMaster.setVoltage(rightSpeed);
  }

  public void drive(double xSpeed, double rot) {
    SmartDashboard.putNumber("move", xSpeed);
    SmartDashboard.putNumber("turn", rot);
    wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    driveMotors(wheelSpeeds);
  }

  public void updateOdometry() {
    m_odometry.update(Rotation2d.fromDegrees(getYaw()), getLeftEncoderDistance(), getRightEncoderDistance());
  }
}