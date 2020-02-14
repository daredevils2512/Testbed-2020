/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SimpleDrivetrain;
import frc.robot.vision.PiTable;

public class FollowBall extends CommandBase {
  private SimpleDrivetrain m_drivetrain;
  private PiTable m_table;
  private NetworkTable m_networkTable;

  private double move = 0.0;
  private double turn = 0.0;
  private double k_moveSpeed;
  private double k_turnSpeed;
  /**
   * Creates a new FollowBall.
   */
  public FollowBall(SimpleDrivetrain drivetrain, PiTable table) {
    m_drivetrain = drivetrain;
    m_table = table;
    m_networkTable = NetworkTableInstance.getDefault().getTable("follow ball");

    m_networkTable.getEntry("k_move").setDouble(k_moveSpeed);
    m_networkTable.getEntry("k_turn").setDouble(k_turnSpeed);
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    k_moveSpeed = m_networkTable.getEntry("k_move").getDouble(0.0);
    k_turnSpeed = m_networkTable.getEntry("k_turn").getDouble(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_table.hasTarget()) {
      move = m_table.getDistance(m_table.getClosestTarget()) * k_moveSpeed;
      turn = m_table.getXOffset(m_table.getClosestTarget()) * k_turnSpeed;
    }
    m_networkTable.getEntry("move").setDouble(move);
    m_networkTable.getEntry("turn").setDouble(turn);
    m_drivetrain.arcadeDrive(move, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
