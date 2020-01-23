/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
// all shifters commented out as testbed has no shifters
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final int m_encoderResolution = 4096; // Just a guess
  private final double m_gearRatio = 4 / 1;
  private final double m_wheelCircumference = 4 * Math.PI;

  private final int m_leftDriveMasterID = 1;
  private final int m_leftDrive1ID = 2;
  private final int m_rightDriveMasterID = 4;
  private final int m_rightDrive1ID = 6;

  private final int m_leftEncoderChannelA = 2; 
  private final int m_leftEncoderChannelB = 3; 
  private final int m_rightEncoderChannelA = 0;
  private final int m_rightEncoderChannelB = 1; 

  // private final int m_shifterForwardChannel = 0;
  // private final int m_shifterReverseChannel = 1;
  // private final DoubleSolenoid.Value m_highGearValue = Value.kForward;
  // private final DoubleSolenoid.Value m_lowGearValue = Value.kReverse;

  protected final WPI_TalonSRX m_leftDriveMaster;
  protected final WPI_TalonSRX m_leftDrive1;
  protected final WPI_TalonSRX m_rightDriveMaster;
  protected final WPI_TalonSRX m_rightDrive1;

  protected Encoder m_leftEncoder;
  protected Encoder m_rightEncoder;

  private final DifferentialDrive m_differentialDrive;

  // private final DoubleSolenoid m_shifter;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    m_leftDriveMaster = new WPI_TalonSRX(m_leftDriveMasterID);
    m_leftDrive1 = new WPI_TalonSRX(m_leftDrive1ID);
    m_rightDriveMaster = new WPI_TalonSRX(m_rightDriveMasterID);
    m_rightDrive1 = new WPI_TalonSRX(m_rightDrive1ID);

    m_leftDrive1.follow(m_leftDriveMaster);
    m_rightDrive1.follow(m_rightDriveMaster);

    // m_leftDriveMaster.configSelectedFeedbackDevice(FeedbackDevice.);

    m_differentialDrive = new DifferentialDrive(m_leftDriveMaster, m_rightDriveMaster);

    // m_shifter = new DoubleSolenoid(m_shifterForwardChannel, m_shifterReverseChannel);

  }

  public void init() {
    m_rightEncoder = new Encoder(m_rightEncoderChannelA, m_rightEncoderChannelB);
    m_leftEncoder = new Encoder(m_leftEncoderChannelA, m_leftEncoderChannelB);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left ticks", getLeftEncoderTicks());
    SmartDashboard.putNumber("left inches", encoderTicksToInches(getLeftEncoderTicks()));
    SmartDashboard.putNumber("right ticks", getRightEncoderTicks());
    SmartDashboard.putNumber("right inches", encoderTicksToInches(getRightEncoderTicks()));
  }

  public void arcadeDrive(double move, double turn) {
    m_differentialDrive.arcadeDrive(move, turn);
  }
  
  // /**
  //  * @return left current[0] and right current[1]
  //  */
  // public double[] getCurrent() {
  //   double[] outputCurrent = new double[2];
  //   outputCurrent[0] = m_leftDrive1.getStatorCurrent();
  //   outputCurrent[1] = m_rightDrive1.getStatorCurrent();
  //   return outputCurrent;
  // }

  // public boolean getLowGear() {
  //   return m_shifter.get() == m_lowGearValue;
  // }

  // public void setLowGear(boolean wantsLowGear) {
  //   m_shifter.set(wantsLowGear ? m_lowGearValue : m_highGearValue);
  // }

  // /**
  //  * drives forward using motion magic
  //  * @param distance distance to go in inches
  // */
  // public void driveTo(final double distance) {
  //   m_leftDriveMaster.set(ControlMode.MotionMagic, inchesToEncoderTicks(distance));
  //   m_rightDriveMaster.set(ControlMode.MotionMagic, inchesToEncoderTicks(distance));
  // }

  public int getLeftEncoderTicks() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderTicks() {
    return m_rightEncoder.get();
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double encoderTicksToInches(int number) {
    return (double)number / m_encoderResolution * m_gearRatio * m_wheelCircumference;
  }

  public int inchesToEncoderTicks(double inches) {
    return (int)(inches / m_wheelCircumference / m_gearRatio * m_encoderResolution);
  }
  // public boolean getLowGear() {
  //   return m_shifter.get() == m_lowGearValue;
  // }

  // public void setLowGear(boolean wantsLowGear) {
  //   m_shifter.set(wantsLowGear ? m_lowGearValue : m_highGearValue);
  // }
}
