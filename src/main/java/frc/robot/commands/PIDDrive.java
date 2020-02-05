package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.KinematicsDrivetrain;

public class PIDDrive extends CommandBase {
  private final KinematicsDrivetrain m_drivetrain;
  private final DoubleSupplier m_moveSupplier;
  private final DoubleSupplier m_turnSupplier;

  private final SlewRateLimiter m_accelerationLimiter;
  private final SlewRateLimiter m_angularAccelerationLimiter;

  /**
   * Velocity control arcade drive
   * 
   * <p> The move and turn is scaled by the drivetrain's maximum
   * linear and angular velocity, respectively
   * @param drivetrain
   * @param moveSupplier
   * @param turnSupplier
   */
  public PIDDrive(KinematicsDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    m_moveSupplier = moveSupplier;
    m_turnSupplier = turnSupplier;
    m_accelerationLimiter = new SlewRateLimiter(10);
    m_angularAccelerationLimiter = new SlewRateLimiter(10);
  }

  @Override
  public void execute() {
    double velocity = m_accelerationLimiter.calculate(m_moveSupplier.getAsDouble()) * m_drivetrain.getMaxSpeed();
    double angularVelocity = m_angularAccelerationLimiter.calculate(m_turnSupplier.getAsDouble()) * m_drivetrain.getMaxAngularSpeed();
    m_drivetrain.velocityArcadeDrive(velocity, angularVelocity);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}