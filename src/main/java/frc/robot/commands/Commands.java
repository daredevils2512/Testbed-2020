package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ClosedLoopDrivetrain;
import frc.robot.subsystems.FalconTest;
import frc.robot.subsystems.SimpleDrivetrain;

public class Commands {
  private static class SetFalconPositionCommand extends CommandBase {
    private FalconTest m_falconTest;
    private double m_position;

    public SetFalconPositionCommand(FalconTest falconTest, double position) {
      m_falconTest = falconTest;
      m_position = position;
      addRequirements(falconTest);
    }

    @Override
    public void initialize() {
      m_falconTest.setPosition(m_position);
    }

    @Override
    public boolean isFinished() {
      return m_falconTest.withinClosedLoopPositionErrorMargin();
    }
  }

  private static class RotateFalconCommand extends CommandBase {
    private FalconTest m_falconTest;
    private double m_rotations;

    public RotateFalconCommand(FalconTest falconTest, double rotations) {
      m_falconTest = falconTest;
      m_rotations = rotations;
      addRequirements(falconTest);
    }

    @Override
    public void initialize() {
      m_falconTest.rotateDistance(m_rotations);
    }

    @Override
    public boolean isFinished() {
      return m_falconTest.withinClosedLoopPositionErrorMargin();
    }
  }

  private static class SetFalconVelocityCommand extends CommandBase {
    private FalconTest m_falconTest;
    private double m_velocity;

    public SetFalconVelocityCommand(FalconTest falconTest, double velocity) {
      m_falconTest = falconTest;
      m_velocity = velocity;
      addRequirements(falconTest);
    }

    @Override
    public void initialize() {
      m_falconTest.rotateDistance(m_velocity);
    }

    @Override
    public boolean isFinished() {
      return m_falconTest.withinClosedLoopVelocityErrorMargin();
    }
  }

  private Commands() {

  }

  public static Command runFalcon(FalconTest falconTest, DoubleSupplier speedSupplier) {
    return new RunCommand(() -> falconTest.run(speedSupplier.getAsDouble()), falconTest);
  }

  public static Command setFalconPosition(FalconTest falconTest, double position) {
    return new SetFalconPositionCommand(falconTest, position);
  }

  /**
   * Set the Falcon's target position for closed loop position control
   * 
   * <p>Interrupted when the target is reached
   * @param falconTest {@link FalconTest} to use
   * @param rotations Distance to rotate
   * @return New {@link Command}
   */
  public static Command rotateFalcon(FalconTest falconTest, double rotations) {
    return new RotateFalconCommand(falconTest, rotations);
  }

  /**
   * Set the Falcon's target velocity for closed loop velocity control
   * @param falconTest {@link FalconTest} to use
   * @param velocity Velocity setpoint in rotations per second
   * @return New {@link Command}
   */
  public static Command setFalconVelocity(FalconTest falconTest, double velocity) {
    return new SetFalconVelocityCommand(falconTest, velocity);
  }

  public static Command arcadeDrive(SimpleDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    return new RunCommand(() -> drivetrain.arcadeDrive(moveSupplier.getAsDouble(), turnSupplier.getAsDouble()), drivetrain);
  }

  public static Command arcadeDriveClosedLoop(ClosedLoopDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    DoubleSupplier velocitySupplier = () -> moveSupplier.getAsDouble() * drivetrain.getMaxSpeed();
    DoubleSupplier angularVelocitySupplier = () -> turnSupplier.getAsDouble() * drivetrain.getMaxAngularSpeed();
    return new RunCommand(() -> drivetrain.arcadeDriveClosedLoop(velocitySupplier.getAsDouble(), angularVelocitySupplier.getAsDouble()), drivetrain);
  }

  public static Command turnToAngle(ClosedLoopDrivetrain drivetrain, double targetAngle, double maxAngularSpeed) {
    return new TurnToAngle(drivetrain, targetAngle, maxAngularSpeed);
  }

  public static Command driveStaight(ClosedLoopDrivetrain drivetrain, double distance, double maxSpeed, double maxAngularSpeed) {
    return new DriveStraight(drivetrain, distance, maxSpeed, maxAngularSpeed);
  }
}