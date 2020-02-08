/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DareMath;
import frc.robot.vision.LimelightUtil;
import frc.robot.vision.Pipeline;

public class FollowBall extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Pipeline m_pipeline;

  //constants to change the moving
  private final double m_maxSpeed = 0.5;
  private final double m_maxTurnSpeed = 0.5;
  private final double m_idleTurnSpeed = 0.2;
  private final double m_targetFill = 0.7;

  private double m_lastHorizontalOffset; // Last detected horizontal offset

  public FollowBall(Drivetrain drivetrain, Pipeline pipeline) {
    m_pipeline = pipeline;
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double move;
    double turn;

    if (LimelightUtil.hasTarget(m_pipeline)) {
      // Speed is inversely proportional to the target fill
      move = DareMath.mapRange(
        m_targetFill - LimelightUtil.getFill(m_pipeline),
        0, 1,
        -m_maxSpeed, m_maxSpeed);

      // Turn speed is inversely proportional to horizontal offset
      turn = DareMath.mapRange(
        -LimelightUtil.getHorizontalOffset(m_pipeline),
        -LimelightUtil.MAX_HORIZONTAL_OFFSET, LimelightUtil.MAX_HORIZONTAL_OFFSET,
        -m_maxTurnSpeed, m_maxTurnSpeed);

      // Record the current horizontal offset in case the target is lost
      m_lastHorizontalOffset = LimelightUtil.getHorizontalOffset(m_pipeline);
    } else {
      // Turn towards offset of last detected target
      turn = m_idleTurnSpeed * -Math.signum(m_lastHorizontalOffset);
      move = 0;
    }
    m_drivetrain.arcadeDrive(move, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
