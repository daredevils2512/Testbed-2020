package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private final NetworkTable m_networkTable;

  private final int m_turretMasterID = 6;
  private final TalonSRX m_turretMaster;

  // TODO: Find encoder and gearing details for turret
  private final double m_encoderResolution = 4096;
  private final double m_gearRatio = 22.0 / 129.0;

  private final double m_maxTurnDegrees = 180;
  private final double m_tolerance = 5;

  // TODO: Tune position PID
  private final int m_positionSlot = 0;
  private double m_P = 8;
  private double m_I = 0;
  private double m_D = 35;
  private int m_motionAcceleration = 2000;
  private int m_motionCruiseVelocity = 4000;

  /**
   * Creates a new turret
   */
  public Turret() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());

    m_turretMaster = new TalonSRX(m_turretMasterID);
    m_turretMaster.configFactoryDefault();

    m_turretMaster.config_IntegralZone(m_positionSlot, 0);
    m_turretMaster.config_kD(m_positionSlot, m_D);
    m_turretMaster.config_kI(m_positionSlot, m_I);
    m_turretMaster.config_kP(m_positionSlot, m_P);
    m_turretMaster.configMotionAcceleration(m_motionAcceleration);
    m_turretMaster.configMotionCruiseVelocity(m_motionCruiseVelocity);

    m_turretMaster.configClosedLoopPeakOutput(m_positionSlot, 1.0);
    m_turretMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    m_turretMaster.setNeutralMode(NeutralMode.Brake);
    m_turretMaster.set(ControlMode.PercentOutput, 0);
    m_turretMaster.setSelectedSensorPosition(0);

    m_turretMaster.configForwardSoftLimitThreshold(toEncoderPulses(m_maxTurnDegrees));
    m_turretMaster.configReverseSoftLimitThreshold(toEncoderPulses(-m_maxTurnDegrees));
    m_turretMaster.configForwardSoftLimitEnable(true);
    m_turretMaster.configReverseSoftLimitEnable(true);

    m_networkTable.getEntry("P gain").setNumber(m_P);
    m_networkTable.getEntry("I gain").setNumber(m_I);
    m_networkTable.getEntry("D gain").setNumber(m_D);
  }

  @Override
  public void periodic() {
    m_networkTable.getEntry("Encoder position").setNumber(getPosition());
    m_P = m_networkTable.getEntry("P gain").getDouble(0.0);
    m_I = m_networkTable.getEntry("I gain").getDouble(0.0);
    m_D = m_networkTable.getEntry("D gain").getDouble(0.0);
    m_turretMaster.config_kD(m_positionSlot, m_D);
    m_turretMaster.config_kI(m_positionSlot, m_I);
    m_turretMaster.config_kP(m_positionSlot, m_P);

    SmartDashboard.putNumber("turret angle", getAngle());
    SmartDashboard.putNumber("turrent ticks", getPosition());
    SmartDashboard.putNumber("turret angle in ticks", toEncoderPulses(getAngle()));
  }

  private int getPosition() {
    return m_turretMaster.getSelectedSensorPosition();
  }

  /**
   * Get the current angle of the turret
   * @return Angle in degrees
   */
  public double getAngle() {
    // Convert from encoder pulses to degrees
    return toDegrees(getPosition());
  }

  public void resetEncoder() {
    m_turretMaster.setSelectedSensorPosition(0);
  }

  public void setSpeed(double speed) {
    m_turretMaster.set(ControlMode.PercentOutput, speed);
  }

  public void runPosition(double degrees) {
    if (Math.abs(getAngle() - degrees) >= m_tolerance) {
      m_turretMaster.set(ControlMode.MotionMagic, 
        toEncoderPulses(wrapDegrees(degrees)));
    }
  }

  public double wrapDegrees(double degrees) {
    return ((degrees + Math.signum(degrees) * m_maxTurnDegrees) % 360) - Math.signum(degrees) * m_maxTurnDegrees;
  }

  /**
   * Set a target angle for position PID
   * @param angle Angle in degrees
   */
  public void setTargetAngle(double angle) {
    m_turretMaster.set(ControlMode.Position, toEncoderPulses(angle));
  }

  private double toDegrees(int encoderPulses) {
    // return ((double)encoderPulses / m_encoderResolution) * 360 * m_gearRatio;
    return ((double)encoderPulses / m_encoderResolution) * 360 * m_gearRatio;
  }

  //returns a fused heading problaby
  private int toEncoderPulses(double angle) {
    return (int)(angle / (360 * m_gearRatio) *  m_encoderResolution);
  }
}