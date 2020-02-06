package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber2020 extends SubsystemBase {
  private final NetworkTable m_networkTable;
  private final NetworkTableEntry m_leftArmLengthEntry;
  private final NetworkTableEntry m_rightArmLengthEntry;

  private final int m_leftExtendMotorID = -1;
  private final int m_rightExtendMotorID = -1;

  private final WPI_TalonSRX m_leftExtendMotor;
  private final WPI_TalonSRX m_rightExtendMotor;

  private final int m_leftActuatorForwardChannel = -1;
  private final int m_leftActuatorReverseChannel = -1;
  private final int m_rightActuatorForwardChannel = -1;
  private final int m_rightActuatorReverseChannel = -1;
  private final DoubleSolenoid.Value m_leftActuatorDownValue = Value.kForward;
  private final DoubleSolenoid.Value m_leftActuatorUpValue = Value.kReverse;
  private final DoubleSolenoid.Value m_rightActuatorDownValue = Value.kForward;
  private final DoubleSolenoid.Value m_rightActuatorUpValue = Value.kReverse;
  private final DoubleSolenoid m_leftActuateSolenoid;
  private final DoubleSolenoid m_rightActuateSolenoid;

  public Climber2020() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());
    m_leftArmLengthEntry = m_networkTable.getEntry("Left arm length");
    m_rightArmLengthEntry = m_networkTable.getEntry("Right arm length");

    m_leftExtendMotor = new WPI_TalonSRX(m_leftExtendMotorID);
    m_rightExtendMotor = new WPI_TalonSRX(m_rightExtendMotorID);

    m_leftExtendMotor.configFactoryDefault();
    m_rightExtendMotor.configFactoryDefault();

    m_leftExtendMotor.setInverted(InvertType.InvertMotorOutput);
    m_rightExtendMotor.setInverted(InvertType.None);

    m_leftActuateSolenoid = new DoubleSolenoid(m_leftActuatorForwardChannel, m_leftActuatorReverseChannel);
    m_rightActuateSolenoid = new DoubleSolenoid(m_rightActuatorForwardChannel, m_rightActuatorReverseChannel);
  }

  @Override
  public void periodic() {
    m_leftArmLengthEntry.setNumber(0);
    m_rightArmLengthEntry.setNumber(0);
  }

  /**
   * Extend/retract the left climber arm using the designated climber motor
   * @param speed
   */
  public void runLeftExtender(double speed) {
    m_leftExtendMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Extend/retract the right climber arm using the designated climber motor
   * @param speed
   */
  public void runRightExtender(double speed) {
    m_rightExtendMotor.set(ControlMode.PercentOutput, speed);
  }
}