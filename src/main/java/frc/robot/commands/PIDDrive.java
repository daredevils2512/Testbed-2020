package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDDrivetrain;

public class PIDDrive extends CommandBase {
  private final PIDDrivetrain m_PIDdrivetrain;
  private final DoubleSupplier m_moveSupplier;
  private final DoubleSupplier m_turnSupplier;

  private final SlewRateLimiter moveLimiter;
  private final SlewRateLimiter turnLimiter;

  public PIDDrive(PIDDrivetrain PIDdrivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    m_moveSupplier = moveSupplier;
    m_turnSupplier = turnSupplier;
    moveLimiter = new SlewRateLimiter(10);
    turnLimiter = new SlewRateLimiter(10);
    m_PIDdrivetrain = PIDdrivetrain;
    addRequirements(PIDdrivetrain);
  }

  @Override
  public void execute() {
    double forward = -moveLimiter.calculate(m_moveSupplier.getAsDouble() * m_PIDdrivetrain.k_maxSpeed);
    double turn = turnLimiter.calculate(m_turnSupplier.getAsDouble() * m_PIDdrivetrain.k_maxTurn);
    m_PIDdrivetrain.drive(forward, turn);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}