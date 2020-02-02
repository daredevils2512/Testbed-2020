/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.ClosedLoopDrivetrain;

public class DriveStraight extends CommandBase {
  private static final double m_distancePGain = 0.5;
  private static final double m_distanceIGain = 0;
  private static final double m_distanceDGain = 0;
  private static final double m_anglePGain = 0.5;
  private static final double m_angleIGain = 0;
  private static final double m_angleDGain = 0;
  private static final double m_distanceTolerance = 0.1; // Distance in meters
  private static final double m_velocityTolerance = 0.01; // Velocity in meters per second
  private static final double m_angleTolerance = 10; // Angle in degrees
  private static final double m_angularVelocityTolerance = 1; // Angular velocity in degrees per second

  private final ClosedLoopDrivetrain m_drivetrain;
  private final Pose2d m_endingPose;

  private final double m_maxSpeed; // Speed in meters per second
  private final double m_maxAngularSpeed; // Angular speed in radians per second

  private final PIDController m_distanceController;
  private final PIDController m_angleController;

  /**
   * Creates a new DriveStraight.
   */
  public DriveStraight(ClosedLoopDrivetrain drivetrain, double distance, double maxSpeed, double maxAngularSpeed) {
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_maxSpeed = maxSpeed;
    m_maxAngularSpeed = maxAngularSpeed;

    Translation2d translation = new Translation2d(distance, 0);
    Transform2d transform = new Transform2d(translation, Rotation2d.fromDegrees(0));
    m_endingPose = drivetrain.getPose().transformBy(transform);

    m_distanceController = new PIDController(m_distancePGain, m_distanceIGain, m_distanceDGain);
    m_angleController = new PIDController(m_anglePGain, m_angleIGain, m_angleDGain);

    m_distanceController.setTolerance(m_distanceTolerance, m_velocityTolerance);
    m_angleController.setTolerance(m_angleTolerance, m_angularVelocityTolerance);

    // The error is calculated using some funky linear algebra
    // in execute which the PID controllers can't handle,
    // so the error is passed in as the measurement and the
    // the setpoint is 0
    m_distanceController.setSetpoint(0);
    m_angleController.setSetpoint(0);
  }

  @Override
  public void execute() {
    // Get the transform from the current pose to the target pose, using the the translation along the X axis as a distance error
    Transform2d transform = m_endingPose.minus(m_drivetrain.getPose()); // Transform from robot pose to target pose
    double distanceControllerOutput = MathUtil.clamp(m_distanceController.calculate(transform.getTranslation().getX()), -1, 1);

    if(!m_distanceController.atSetpoint()) {
      // Get the rotation from the current translation to the target translation
      Translation2d translation = m_endingPose.minus(m_drivetrain.getPose()).getTranslation();
      Rotation2d rotation = new Rotation2d(translation.getX(), translation.getY()).minus(m_drivetrain.getPose().getRotation()).minus(m_drivetrain.getPose().getRotation());
      double angleControllerOutput = MathUtil.clamp(m_angleController.calculate(rotation.getDegrees()), -1, 1);
      
      double velocity = (distanceControllerOutput - angleControllerOutput) * m_maxSpeed;
      double angularVelocity = angleControllerOutput * m_maxAngularSpeed;
      m_drivetrain.arcadeDriveClosedLoop(velocity, angularVelocity);
    } else {
      double angleControllerOutput = MathUtil.clamp(m_angleController.calculate(transform.getRotation().getDegrees()), -1, 1);
      double angularVelocity = angleControllerOutput * m_maxAngularSpeed;
      m_drivetrain.arcadeDriveClosedLoop(0, angularVelocity);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_distanceController.atSetpoint() && m_angleController.atSetpoint();
  }
}
