/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.KinematicsDrivetrain;
import frc.robot.subsystems.OdometryDrivetrain;

public class TurnToAngle<T extends KinematicsDrivetrain & OdometryDrivetrain> extends CommandBase {
  private final T m_drivetrain;
  private final PIDController m_angleController;

  private final double m_pGain = 0.5;
  private final double m_iGain = 0;
  private final double m_dGain = 0;
  private final double m_angleTolerance = 5; // Degrees
  private final double m_angularSpeedTolerance = 1; // Degrees per second
  private final double m_maxAngularSpeed;

  /**
   * Turn the robot to a set angle using PID
   * @param drivetrain
   * @param targetAngle Angle in degrees
   */
  public TurnToAngle(T drivetrain, double targetAngle, double maxAngularSpeed) {
    addRequirements(drivetrain);

    m_angleController = new PIDController(m_pGain, m_iGain, m_dGain);
    m_angleController.enableContinuousInput(-180, 180);
    m_angleController.setTolerance(m_angleTolerance, m_angularSpeedTolerance);
    m_angleController.setSetpoint(targetAngle);
    
    m_drivetrain = drivetrain;
    m_maxAngularSpeed = maxAngularSpeed;

  }

  @Override
  public void execute() {
    double pidOutput = m_angleController.calculate(m_drivetrain.getPose().getRotation().getDegrees());
    double angularVelocity = pidOutput * m_maxAngularSpeed;
    m_drivetrain.velocityArcadeDrive(0, angularVelocity);
  }

  @Override
  public boolean isFinished() {
    return m_angleController.atSetpoint();
  }
}
