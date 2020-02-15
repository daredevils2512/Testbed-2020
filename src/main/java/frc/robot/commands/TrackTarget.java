package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.vision.LimelightUtil;
import frc.robot.vision.Pipeline;

public class TrackTarget extends CommandBase {
  private final Turret m_turret;
  private final Pipeline m_pipeline;
  private final Supplier<Pose2d> m_robotPoseSupplier;

  public TrackTarget(Turret turret, Pipeline pipeline, Supplier<Pose2d> robotPoseSupplier) {
    m_turret = turret;
    m_pipeline = pipeline;
    m_robotPoseSupplier = robotPoseSupplier;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    if (LimelightUtil.hasTarget(m_pipeline)) {
      // Negated because limelight is CW positive, but everything else is CCW positive
      // double targetOffset = -LimelightUtil.getHorizontalOffset(m_pipeline);
      // double targetDistance = LimelightUtil.calculateDistance(m_pipeline, 3, Limelight.TURRET.getAngleOfElevation());
      
      // double newTurretAngle = m_turret.getAngle() + targetOffset;

      // if (newTurretAngle > m_turret.getMaxAngle())
      //   newTurretAngle -= 360;
      // else if (newTurretAngle < m_turret.getMinAngle())
      //   newTurretAngle += 360;
      
      // m_turret.setTargetAngle(newTurretAngle);
    } else {
      m_turret.setSpeed(0);
    }
  }
}