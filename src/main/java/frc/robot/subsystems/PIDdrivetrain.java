package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDdrivetrain extends SubsystemBase {
  protected PIDController m_leftPIDdrive;
  protected PIDController m_rightPIDdrive;

  private NetworkTable m_networkTable;

  private final int m_leftDriveMasterID = 1;
  private final int m_leftDrive1ID = 1;
  private final int m_rightDriveMasterID = 1;
  private final int m_rightDrive1ID = 1;

  private final int m_leftEncoderChannelA = 2;
  private final int m_leftEncoderChannelB = 3;
  private final int m_rightEncoderChannelA = 0;
  private final int m_rightEncoderChannelB = 1;

  private double k_lP = 0.0; //left PID
  private double k_lI = 0.0; //TODO: tune these
  private double k_lD = 0.0;

  private double k_rP = 0.0; //right PID
  private double k_rI = 0.0;
  private double k_rD = 0.0;
  
  private final TalonSRX m_leftDriveMaster;
  private final TalonSRX m_leftDrive1;
  private final TalonSRX m_rightDriveMaster;
  private final TalonSRX m_rightDrive1;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  public PIDdrivetrain(PIDController PIDdrive, NetworkTable netowrkTable) {
    m_networkTable = netowrkTable;

    m_leftDriveMaster = new TalonSRX(m_leftDriveMasterID);
    m_leftDrive1 = new TalonSRX(m_leftDrive1ID);
    m_rightDriveMaster = new TalonSRX(m_rightDriveMasterID);
    m_rightDrive1 = new TalonSRX(m_rightDrive1ID);

    m_leftDrive1.follow(m_leftDriveMaster);
    m_rightDrive1.follow(m_rightDriveMaster);

    m_leftEncoder = new Encoder(m_leftEncoderChannelA, m_leftEncoderChannelB);
    m_rightEncoder = new Encoder(m_rightEncoderChannelA, m_rightEncoderChannelB);

    m_leftPIDdrive = new PIDController(k_lP, k_lI, k_lD);
    m_rightPIDdrive = new PIDController(k_rP, k_rI, k_rD);
    m_leftPIDdrive.setTolerance(0.1);
    m_rightPIDdrive.setTolerance(0.1);

    m_networkTable.getEntry("left drivetrain P").setNumber(k_lP);
    m_networkTable.getEntry("left drivetrain I").setNumber(k_lI);
    m_networkTable.getEntry("left drivetrain D").setNumber(k_lD);
    m_networkTable.getEntry("right drivetrian P").setNumber(k_rP);
    m_networkTable.getEntry("right drivetrian I").setNumber(k_rI);
    m_networkTable.getEntry("right drivetrian D").setNumber(k_rD);
  }

  public class RightPIDdrive extends PIDSubsystem {
    public RightPIDdrive() {
      super(m_rightPIDdrive);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
      m_rightDriveMaster.set(ControlMode.PercentOutput, output);
    }

    @Override
    protected double getMeasurement() {
      return m_rightEncoder.getDistance();
    }
  }

  public class LeftPIDdrive extends PIDSubsystem {
    public LeftPIDdrive() {
      super(m_leftPIDdrive);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
      m_leftDriveMaster.set(ControlMode.PercentOutput, output);
    }

    @Override
    protected double getMeasurement() {
      return m_leftEncoder.getDistance();
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
    m_leftPIDdrive.setPID(k_lP, k_lI, k_lD);
    m_rightPIDdrive.setPID(k_rP, k_rI, k_rD);
    super.periodic(); 
  }
}