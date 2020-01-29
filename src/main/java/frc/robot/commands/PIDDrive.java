package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.PIDdrivetrain;

public class PIDDrive extends CommandBase {
  private final PIDdrivetrain m_PIDdrivetrain;
  private double m_forward;
  private double m_turn;

  private final SlewRateLimiter moveLimiter;
  private final SlewRateLimiter turnLimiter;
  private final ControlBoard m_controlBoard;

  public PIDDrive(PIDdrivetrain PIDdrivetrain, ControlBoard controlBoard) {
    moveLimiter = new SlewRateLimiter(5);
    turnLimiter = new SlewRateLimiter(5);
    m_controlBoard = controlBoard;
    m_PIDdrivetrain = PIDdrivetrain;
    addRequirements(PIDdrivetrain);
  }

  @Override
  public void execute() {
    m_forward = moveLimiter.calculate(m_controlBoard.xbox.getLeftStickY() * m_PIDdrivetrain.k_maxSpeed);
    m_turn = turnLimiter.calculate(m_controlBoard.xbox.getRightStickX() * m_PIDdrivetrain.k_maxTurn);
    m_PIDdrivetrain.drive(m_forward, m_turn);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}