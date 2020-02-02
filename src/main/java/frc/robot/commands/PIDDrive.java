package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClosedLoopDrivetrain;

public class PIDDrive extends CommandBase {
  private final ClosedLoopDrivetrain m_drivetrain;
  private final DoubleSupplier m_moveSupplier;
  private final DoubleSupplier m_turnSupplier;

  private final SlewRateLimiter m_moveLimiter;
  private final SlewRateLimiter m_turnLimiter;

  public PIDDrive(ClosedLoopDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    m_moveSupplier = moveSupplier;
    m_turnSupplier = turnSupplier;
    m_moveLimiter = new SlewRateLimiter(10);
    m_turnLimiter = new SlewRateLimiter(10);
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    double forward = m_moveLimiter.calculate(m_moveSupplier.getAsDouble() * m_drivetrain.getMaxSpeed());
    double turn = m_turnLimiter.calculate(m_turnSupplier.getAsDouble() * m_drivetrain.getMaxAngularSpeed());
    m_drivetrain.arcadeDriveClosedLoop(forward, turn);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}