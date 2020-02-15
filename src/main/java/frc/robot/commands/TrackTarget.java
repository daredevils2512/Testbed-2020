package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.vision.LimelightUtil;
import frc.robot.vision.Pipeline;

public class TrackTarget extends CommandBase {
  private final Turret m_turret;
  private final Pipeline m_pipeline;
  private final Supplier<Pose2d> m_robotPoseSupplier;

  private double m_lastTargetAngle;
  private Pose2d m_lastRobotPoseWithTarget;

  public TrackTarget(Turret turret, Pipeline pipeline, Supplier<Pose2d> robotPoseSupplier) {
    m_turret = turret;
    m_pipeline = pipeline;
    m_robotPoseSupplier = robotPoseSupplier;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    m_lastTargetAngle = m_turret.getAngle();
    m_lastRobotPoseWithTarget = m_robotPoseSupplier.get();
  }

  @Override
  public void execute() {
    if (LimelightUtil.hasTarget(m_pipeline)) {
      double newTargetAngle = m_turret.getAngle() + getTargetOffset();

      if (newTargetAngle > m_turret.getMaxAngle())
        newTargetAngle -= 360;
      else if (newTargetAngle < m_turret.getMinAngle())
        newTargetAngle += 360;
      
      m_turret.setTargetAngle(newTargetAngle);

      m_lastTargetAngle = getTargetOffset();
      m_lastRobotPoseWithTarget = m_robotPoseSupplier.get();
    } else {
      double robotPoseChangeSinceTargetDetected = m_robotPoseSupplier.get().minus(m_lastRobotPoseWithTarget).getRotation().getDegrees();
      double newTurretAngle = m_lastTargetAngle - robotPoseChangeSinceTargetDetected;
      m_turret.setTargetAngle(newTurretAngle);
    }
  }

  private double getTargetOffset() {
    // Offset of the target to the turret's current angle in degrees
    // Negated because limelight is CW positive, but everything else is CCW positive
    return -LimelightUtil.getHorizontalOffset(m_pipeline);
  }

  public double getTargetDistance() {
    return LimelightUtil.calculateDistance(m_pipeline, Constants.TARGET_CENTER_HEIGHT, m_pipeline.getLimelight().getAngleOfElevation());
  }
}