package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.FalconTest;
import frc.robot.subsystems.drivetrain.KinematicsDrivetrain;
import frc.robot.subsystems.drivetrain.OdometryDrivetrain;
import frc.robot.subsystems.drivetrain.SimpleDrivetrain;

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

  public static Command resetDrivetrainPose(OdometryDrivetrain drivetrain) {
    return new InstantCommand(() -> drivetrain.resetPose(), drivetrain);
  }

  /**
   * Simple percent output arcade drive
   * @param drivetrain Drivetrain to use
   * @param moveSupplier Forward speed supplier (-1 to +1)
   * @param turnSupplier Turn speed supplier (-1 to +1)
   * @return New {@link Command}
   */
  public static Command simpleArcadeDrive(SimpleDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    return new RunCommand(() -> drivetrain.arcadeDrive(moveSupplier.getAsDouble(), turnSupplier.getAsDouble()), drivetrain);
  }

  /**
   * Arcade drive with PID velocity control
   * @param drivetrain Drivetrain to use
   * @param moveSupplier Forward speed supplier (-1 to +1)
   * @param turnSupplier Turn speed supplier (-1 to +1)
   * @return New {@link Command}
   */
  public static Command velocityArcadeDrive(KinematicsDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    DoubleSupplier velocitySupplier = () -> moveSupplier.getAsDouble() * drivetrain.getMaxSpeed();
    DoubleSupplier angularVelocitySupplier = () -> turnSupplier.getAsDouble() * drivetrain.getMaxAngularSpeed();
    return new RunCommand(() -> drivetrain.velocityArcadeDrive(velocitySupplier.getAsDouble(), angularVelocitySupplier.getAsDouble()), drivetrain);
  }

  /**
   * 
   * @param drivetrain
   * @param moveSupplier
   * @param turnSupplier
   * @param maxAcceleration
   * @param maxAngularAcceleration
   * @return
   */
  public static Command accelerationLimitedSimpleArcadeDrive(SimpleDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier, double maxAcceleration, double maxAngularAcceleration) {
    return new AccelerationLimitedSimpleArcadeDrive(drivetrain, moveSupplier, turnSupplier, maxAcceleration, maxAngularAcceleration);
  }

  // public static Command velocityArcadeDrive(KinematicsDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
  //   return new PIDDrive(drivetrain, moveSupplier, turnSupplier);
  // }

  public static Command accelerationLimitedVelocityArcadeDrive(KinematicsDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier, double maxAcceleration, double maxAngularAcceleration) {
    return new AccelerationLimitedVelocityArcadeDrive(drivetrain, moveSupplier, turnSupplier, maxAcceleration, maxAngularAcceleration);
  }

  public static <T extends KinematicsDrivetrain & OdometryDrivetrain> Command turnToAngle(T drivetrain, double targetAngle, double maxAngularSpeed) {
    return new TurnToAngle<T>(drivetrain, targetAngle, maxAngularSpeed);
  }

  public static <T extends KinematicsDrivetrain & OdometryDrivetrain> Command driveStaight(T drivetrain, double distance, double maxSpeed, double maxAngularSpeed) {
    return new DriveStraight<T>(drivetrain, distance, maxSpeed, maxAngularSpeed);
  }
}