package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.drivetrain.KinematicsDrivetrain;
import frc.robot.subsystems.drivetrain.OdometryDrivetrain;
import frc.robot.subsystems.drivetrain.PIDDrivetrain;
import frc.robot.subsystems.drivetrain.SimpleDrivetrain;

public class Commands {
  private Commands() {

  }

  public static Command pidDrive(PIDDrivetrain drivetrian, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    return new RunCommand(() -> drivetrian.drive(moveSupplier.getAsDouble(), turnSupplier.getAsDouble()), drivetrian);
  }

  public static Command simpleArcadeDrive(SimpleDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    return new RunCommand(() -> drivetrain.arcadeDrive(moveSupplier.getAsDouble(), turnSupplier.getAsDouble()), drivetrain);
  }

  public static Command velocityArcadeDrive(KinematicsDrivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
    return new PIDDrive(drivetrain, moveSupplier, turnSupplier);
  }

  public static <T extends KinematicsDrivetrain & OdometryDrivetrain> Command turnToAngle(T drivetrain, double targetAngle, double maxAngularSpeed) {
    return new TurnToAngle<T>(drivetrain, targetAngle, maxAngularSpeed);
  }

  public static <T extends KinematicsDrivetrain & OdometryDrivetrain> Command driveStaight(T drivetrain, double distance, double maxSpeed, double maxAngularSpeed) {
    return new DriveStraight<T>(drivetrain, distance, maxSpeed, maxAngularSpeed);
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
}