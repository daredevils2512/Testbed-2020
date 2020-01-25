package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PIDdrivetrain extends Drivetrain {
  protected PIDController m_leftPIDcontroller;
  protected PIDController m_rightPIDcontroller;

  private PIDdrive leftPIDdrive;
  private PIDdrive rightPIDdrive;
  
  private double k_lP = 5.0; //left PID
  private double k_lI = 0.0; //TODO: tune these
  private double k_lD = 0.0;

  private double k_rP = -5.0; //right PID -- right P has to be negative i think
  private double k_rI = 0.0; 
  private double k_rD = 0.0;

  public PIDdrivetrain() {
    
    m_rightPIDcontroller = new PIDController(k_rP, k_rI, k_rD, 0.2);
    m_leftPIDcontroller = new PIDController(k_lP, k_lI, k_lD, 0.2);

    SmartDashboard.getEntry("left drivetrain P").setNumber(k_lP);
    SmartDashboard.getEntry("left drivetrain I").setNumber(k_lI);
    SmartDashboard.getEntry("left drivetrain D").setNumber(k_lD);
    SmartDashboard.getEntry("right drivetrian P").setNumber(k_rP);
    SmartDashboard.getEntry("right drivetrian I").setNumber(k_rI);
    SmartDashboard.getEntry("right drivetrian D").setNumber(k_rD);

    leftPIDdrive = new PIDdrive(m_leftDriveMaster, m_leftEncoder, m_leftPIDcontroller);
    rightPIDdrive = new PIDdrive(m_rightDriveMaster, m_rightEncoder, m_rightPIDcontroller);
    leftPIDdrive.enable();
    rightPIDdrive.enable();
  }

  private class PIDdrive extends PIDSubsystem {
    private PIDController m_controller;
    private WPI_TalonSRX m_motor;
    private Encoder m_encoder;

    public PIDdrive(WPI_TalonSRX motor, Encoder encoder, PIDController controller) {
      super(controller);
      m_motor = motor;
      m_controller = controller;
      m_encoder = encoder;
      m_controller.setTolerance(0.1);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
      m_motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    protected double getMeasurement() {
      return m_encoder.getDistance();
    }

    @Override
    public void periodic() {
      super.periodic();
    }

    public void setPID(double kP, double kI, double kD) {
      m_controller.setPID(kP, kI, kD);
    }
  }

  @Override
  public void periodic() {
    k_lP = SmartDashboard.getEntry("left drivetrain P").getDouble(0.0);
    k_lI = SmartDashboard.getEntry("left drivetrain I").getDouble(0.0);
    k_lD = SmartDashboard.getEntry("left drivetrain D").getDouble(0.0);
    k_rP = SmartDashboard.getEntry("right drivetrain P").getDouble(0.0);
    k_rI = SmartDashboard.getEntry("right drivetrain I").getDouble(0.0);
    k_rD = SmartDashboard.getEntry("right drivetrain D").getDouble(0.0);
    leftPIDdrive.setPID(k_lP, k_lI, k_lD);
    rightPIDdrive.setPID(k_rP, k_rI, k_rD);
    super.periodic(); 
  }

  /**
   * sets a setpoint in inches; make left and right different for turning
   * @param left left drive
   * @param right right drive
   */
  public void setSetPoint(double left, double right) {
    leftPIDdrive.setSetpoint(left);
    rightPIDdrive.setSetpoint(right);
  }
  
  /**
   * adds a setpoint to its current position to go the amount reletive to its current position
   * @param left left distance
   * @param right right distance
   */
  public void addSetPoint(double left, double right) {
    setSetPoint(leftPIDdrive.getMeasurement() + left, rightPIDdrive.getMeasurement() + right);
  }

  /**
   * arcade drive for PID drivetrain concept
   * @param speed how far foreward it should go in the next period in inches
   * @param turn how much turniness it should do in the next period in inches
   */
  public void PIDarcade(double speed, double turn) {
    addSetPoint(speed + turn, speed - turn);
  }
}