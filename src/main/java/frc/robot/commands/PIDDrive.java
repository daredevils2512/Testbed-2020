package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDdrivetrain;

public class PIDDrive extends CommandBase {
  private final PIDdrivetrain m_PIDdrivetrain;
  private double m_forward;
  private double m_turn;

  public PIDDrive(PIDdrivetrain PIDdrivetrain, double forward, double turn) {
    m_forward = forward;
    m_turn = turn;
    m_PIDdrivetrain = PIDdrivetrain;
    addRequirements(PIDdrivetrain);
  }

  @Override
  public void execute() {
    m_PIDdrivetrain.drive(m_forward, m_turn);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}