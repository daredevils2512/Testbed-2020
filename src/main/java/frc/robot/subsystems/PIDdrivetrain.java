package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PIDdrivetrain extends PIDSubsystem {
  protected edu.wpi.first.wpilibj.controller.PIDController m_PIDdrive;

  private final int m_leftDriveMasterID = 1;
  private final int m_leftDrive1ID = 1;
  private final int m_rightDriveMasterID = 1;
  private final int m_rightDrive1ID = 1;
  private final int m_leftEncoderChannelA = 2;
  private final int m_leftEncoderChannelB = 3;
  private final int m_rightEncoderChannelA = 0;
  private final int m_rightEncoderChannelB = 1;

  private final TalonSRX m_leftDriveMaster;
  private final TalonSRX m_leftDrive1;
  private final TalonSRX m_rightDriveMaster;
  private final TalonSRX m_rightDrive1;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  public PIDdrivetrain(PIDController PIDdrive) {
    super(PIDdrive);
    m_PIDdrive = PIDdrive;

    m_leftDriveMaster = new TalonSRX(m_leftDriveMasterID);
    m_leftDrive1 = new TalonSRX(m_leftDrive1ID);
    m_rightDriveMaster = new TalonSRX(m_rightDriveMasterID);
    m_rightDrive1 = new TalonSRX(m_rightDrive1ID);

    m_leftDrive1.follow(m_leftDriveMaster);
    m_rightDrive1.follow(m_rightDriveMaster);

    m_leftEncoder = new Encoder(m_leftEncoderChannelA, m_leftEncoderChannelB);
    m_rightEncoder = new Encoder(m_rightEncoderChannelA, m_rightEncoderChannelB);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    m_leftDriveMaster.set(ControlMode.PercentOutput, output);
    m_rightDriveMaster.set(ControlMode.PercentOutput, output);
  }

  @Override
  protected double getMeasurement() {
    return 0;
  }

  @Override
  public void setSetpoint(double setpoint) {
    super.setSetpoint(setpoint);
  }
}