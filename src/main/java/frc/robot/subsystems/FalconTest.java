/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FalconTest extends SubsystemBase {
  private final NetworkTable m_networkTable;

  private final int m_falconID = 12;
  private final TalonFX m_motor;

  private final SupplyCurrentLimitConfiguration m_supplyCurrentLimitConfig;

  private final int m_encoderResolution = 2048;

  private final int m_positionPIDSlot = 0;
  private final int m_velocityPIDSlot = 1;

  // Make final when done tuning
  private double m_positionPGain = 0;
  private double m_positionIGain = 0;
  private double m_positionDGain = 0;

  private double m_velocityPGain = 0;
  private double m_velocityIGain = 0;
  private double m_velocityDGain = 0;

  private final int m_allowableClosedLoopPositionError = 100;
  private final int m_allowableClosedLoopVelocityError = 100;

  /**
   * Creates a new FalconTest.
   */
  public FalconTest() {
    m_networkTable = NetworkTableInstance.getDefault().getTable(getName());

    m_supplyCurrentLimitConfig = new SupplyCurrentLimitConfiguration();
    m_supplyCurrentLimitConfig.currentLimit = 60;

    m_motor = new TalonFX(m_falconID);

    // Prevent unexpected behavior
    m_motor.configFactoryDefault();

    m_motor.configSupplyCurrentLimit(m_supplyCurrentLimitConfig);

    m_motor.config_kP(m_positionPIDSlot, m_positionPGain);
    m_motor.config_kI(m_positionPIDSlot, m_positionIGain);
    m_motor.config_kD(m_positionPIDSlot, m_positionDGain);

    m_motor.config_kP(m_velocityPIDSlot, m_velocityPGain);
    m_motor.config_kI(m_velocityPIDSlot, m_velocityIGain);
    m_motor.config_kD(m_velocityPIDSlot, m_velocityDGain);

    m_motor.setNeutralMode(NeutralMode.Coast);
    m_motor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    m_networkTable.getEntry("Supply current").setNumber(m_motor.getSupplyCurrent());
    m_networkTable.getEntry("Output percent").setNumber(m_motor.getMotorOutputPercent());
    m_networkTable.getEntry("Rotations").setNumber(getDistance());
    m_networkTable.getEntry("Velocity").setNumber(getVelocity());

    m_positionPGain = m_networkTable.getEntry("Position P gain").getNumber(m_positionPGain).doubleValue();
    m_positionIGain = m_networkTable.getEntry("Position I gain").getNumber(m_positionIGain).doubleValue();
    m_positionDGain = m_networkTable.getEntry("Position D gain").getNumber(m_positionDGain).doubleValue();

    m_velocityPGain = m_networkTable.getEntry("Velocity P gain").getNumber(m_velocityPGain).doubleValue();
    m_velocityIGain = m_networkTable.getEntry("Velocity I gain").getNumber(m_velocityIGain).doubleValue();
    m_velocityDGain = m_networkTable.getEntry("Velocity D gain").getNumber(m_velocityDGain).doubleValue();

    m_networkTable.getEntry("Position P gain").setNumber(m_positionPGain);
    m_networkTable.getEntry("Position I gain").setNumber(m_positionIGain);
    m_networkTable.getEntry("Position D gain").setNumber(m_positionDGain);

    m_networkTable.getEntry("Velocity P gain").setNumber(m_velocityPGain);
    m_networkTable.getEntry("Velocity I gain").setNumber(m_velocityIGain);
    m_networkTable.getEntry("Velocity D gain").setNumber(m_velocityDGain);

    m_motor.config_kP(m_positionPIDSlot, m_positionPGain);
    m_motor.config_kI(m_positionPIDSlot, m_positionIGain);
    m_motor.config_kD(m_positionPIDSlot, m_positionDGain);

    m_motor.config_kP(m_velocityPIDSlot, m_velocityPGain);
    m_motor.config_kI(m_velocityPIDSlot, m_velocityIGain);
    m_motor.config_kD(m_velocityPIDSlot, m_velocityDGain);
  }

  public boolean withinClosedLoopPositionErrorMargin() {
    double error = m_motor.getClosedLoopError(m_positionPIDSlot);
    return error < m_allowableClosedLoopPositionError;
  }

  public boolean withinClosedLoopVelocityErrorMargin() {
    double error = m_motor.getClosedLoopError(m_velocityPIDSlot);
    return error < m_allowableClosedLoopVelocityError;
  }

  /**
   * Get accumulated distance
   * @return Distance in rotations
   */
  public double getDistance() {
    return toRotations(m_motor.getSelectedSensorPosition());
  }

  public double getVelocity() {
    return toRotationsPerSecond(m_motor.getSelectedSensorVelocity());
  }

  public void run(double speed) {
    m_motor.set(ControlMode.PercentOutput, speed);
  }

  public void setPosition(double position) {
    m_motor.set(ControlMode.Position, toEncoderPulses(position));
  }

  /**
   * Rotate a set distance using closed loop position control
   * @param distance Distance in rotations
   */  
  public void rotateDistance(double distance) {
    m_motor.selectProfileSlot(m_positionPIDSlot, 0);
    m_motor.set(ControlMode.Position, m_motor.getSelectedSensorPosition() + toEncoderPulses(distance));
  }

  /**
   * Set target for closed loop velocity control
   * @param velocity Velocity in rotations per second
   */
  public void setVelocity(double velocity) {
    m_motor.set(ControlMode.Velocity, toEncoderPulsesPer100ms(velocity));
  }

  private double toRotations(int encoderPulses) {
    return (double)encoderPulses / m_encoderResolution;
  }

  private int toEncoderPulses(double rotations) {
    return (int)(rotations * m_encoderResolution);
  }

  private double toRotationsPerSecond(int encoderPulsesPer100ms) {
    // To seconds then to rotations
    return (double)encoderPulsesPer100ms * 10 / m_encoderResolution;
  }

  private double toEncoderPulsesPer100ms(double rotationsPerSecond) {
    // To encoder pulses then to 100ms
    return rotationsPerSecond * m_encoderResolution / 10;
  }
}
