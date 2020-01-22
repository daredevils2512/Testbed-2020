package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PIDdrivetrain extends Drivetrain {
  protected PIDController m_leftPIDcontroller;
  protected PIDController m_rightPIDcontroller;

  private PIDdrive rightPIDdrive;
  private PIDdrive leftPIDdrive;

  private NetworkTable m_networkTable;




  private double k_lP = 0.0; //left PID
  private double k_lI = 0.0; //TODO: tune these
  private double k_lD = 0.0;

  private double k_rP = 0.0; //right PID
  private double k_rI = 0.0;
  private double k_rD = 0.0;

  public PIDdrivetrain(NetworkTable netowrkTable) {

    m_networkTable = netowrkTable;

    m_leftPIDcontroller = new PIDController(k_lP, k_lI, k_lD);
    m_rightPIDcontroller = new PIDController(k_rP, k_rI, k_rD);
    m_leftPIDcontroller.setTolerance(0.1);
    m_rightPIDcontroller.setTolerance(0.1);

    m_networkTable.getEntry("left drivetrain P").setNumber(k_lP);
    m_networkTable.getEntry("left drivetrain I").setNumber(k_lI);
    m_networkTable.getEntry("left drivetrain D").setNumber(k_lD);
    m_networkTable.getEntry("right drivetrian P").setNumber(k_rP);
    m_networkTable.getEntry("right drivetrian I").setNumber(k_rI);
    m_networkTable.getEntry("right drivetrian D").setNumber(k_rD);

    rightPIDdrive = new PIDdrive(m_rightDriveMaster, m_rightEncoder, m_rightPIDcontroller);
    leftPIDdrive = new PIDdrive(m_leftDriveMaster, m_leftEncoder, m_leftPIDcontroller);
  }

  private class PIDdrive extends PIDSubsystem {
    private PIDController m_controller;
    private WPI_TalonSRX m_motor;
    private Encoder m_encoder;

    public PIDdrive(WPI_TalonSRX motor, Encoder encoder, PIDController controller) {
      super(controller);
      m_motor = motor;
      m_controller = controller;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
      m_motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    protected double getMeasurement() {
      return m_encoder.getDistance();
    }
  }

  @Override
  public void periodic() {
    k_lP = m_networkTable.getEntry("left drivetrain P").getDouble(0.0);
    k_lI = m_networkTable.getEntry("left drivetrain I").getDouble(0.0);
    k_lD = m_networkTable.getEntry("left drivetrain D").getDouble(0.0);
    k_rP = m_networkTable.getEntry("right drivetrain D").getDouble(0.0);
    k_rI = m_networkTable.getEntry("right drivetrain D").getDouble(0.0);
    k_rD = m_networkTable.getEntry("right drivetrain D").getDouble(0.0);
    m_leftPIDcontroller.setPID(k_lP, k_lI, k_lD);
    m_rightPIDcontroller.setPID(k_rP, k_rI, k_rD);
    super.periodic(); 
    
  }

  public void driveDistance(double inches) {
    this.setSetPoint(this.inchesToEncoderTicks(inches), this.inchesToEncoderTicks(inches));
  }

  public void setSetPoint(double left, double right) {
    leftPIDdrive.setSetpoint(left);
    rightPIDdrive.setSetpoint(right);
  }
}