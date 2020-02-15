package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.drivetrain.EncoderDrivetrain;
import frc.robot.subsystems.drivetrain.GyroDrivetrain;
import frc.robot.subsystems.drivetrain.KinematicsDrivetrain;
import frc.robot.subsystems.drivetrain.OdometryDrivetrain;
import frc.robot.subsystems.drivetrain.PIDDrivetrain;
import frc.robot.subsystems.drivetrain.SimpleDrivetrain;
import frc.robot.vision.Pipeline;
import frc.robot.vision.Limelight;

public class Commands {
  private Commands() {

  }

  public static Command pidDrive(PIDDrivetrain drivetrian, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    return new RunCommand(() -> drivetrian.drive(moveSupplier.getAsDouble(), turnSupplier.getAsDouble()), drivetrian);
  }
  
  public static Command resetEncoders(EncoderDrivetrain drivetrain) {
    return new InstantCommand(() -> drivetrain.resetDriveEncoders(), drivetrain);
  }

  public static Command resetHeading(GyroDrivetrain drivetrain) {
    return new InstantCommand(() -> drivetrain.resetHeading(), drivetrain);
  }

  public static Command resetPose(OdometryDrivetrain drivetrain) {
    return new InstantCommand(() -> drivetrain.resetPose());
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

  public static Command accelerationLimitedVelocityArcadeDrive(KinematicsDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier, double maxAcceleration, double maxAngularAcceleration) {
    return new AccelerationLimitedVelocityArcadeDrive(drivetrain, moveSupplier, turnSupplier, maxAcceleration, maxAngularAcceleration);
  }

  public static Command runTurret(Turret turret, DoubleSupplier turnSupplier) {
    return new RunCommand(() -> turret.setSpeed(turnSupplier.getAsDouble()), turret);
  }

  public static Command runTurretPID(Turret turret, double angle) {
    return new RunCommand(() -> turret.runPosition(angle), turret);
  }

  public static Command resetTurret(Turret turret) {
    return new RunCommand(() -> turret.resetEncoder(), turret);
  }

  public static Command findTarget(Turret turret, Limelight limelight, double tolerance) {
    return new FindTarget(turret, tolerance);
  }

  public static Command followBall(KinematicsDrivetrain drivetrain, Pipeline pipeline) {
    return new FollowBall(drivetrain, pipeline);
  }
}