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

  private PIDdrive rightPIDdrive;
  private PIDdrive leftPIDdrive;

  private double k_lP = 0.0; //left PID
  private double k_lI = 0.0; //TODO: tune these
  private double k_lD = 0.0;

  private double k_rP = 0.0; //right PID
  private double k_rI = 0.0;
  private double k_rD = 0.0;

  public PIDdrivetrain() {

    m_leftPIDcontroller = new PIDController(k_lP, k_lI, k_lD);
    m_rightPIDcontroller = new PIDController(k_rP, k_rI, k_rD);
    m_leftPIDcontroller.setTolerance(0.1);
    m_rightPIDcontroller.setTolerance(0.1);

    SmartDashboard.getEntry("left drivetrain P").setNumber(k_lP);
    SmartDashboard.getEntry("left drivetrain I").setNumber(k_lI);
    SmartDashboard.getEntry("left drivetrain D").setNumber(k_lD);
    SmartDashboard.getEntry("right drivetrian P").setNumber(k_rP);
    SmartDashboard.getEntry("right drivetrian I").setNumber(k_rI);
    SmartDashboard.getEntry("right drivetrian D").setNumber(k_rD);

    rightPIDdrive = new PIDdrive(m_rightDriveMaster, m_rightEncoder, m_rightPIDcontroller);
    leftPIDdrive = new PIDdrive(m_leftDriveMaster, m_leftEncoder, m_leftPIDcontroller);
    rightPIDdrive.setName("right PID drivetrain");
    leftPIDdrive.setName("left PID drivetrain");
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

  public void setSetPoint(double left, double right) {
    rightPIDdrive.enable();
    leftPIDdrive.enable();
    leftPIDdrive.setSetpoint(left);
    rightPIDdrive.setSetpoint(right);
  }
}