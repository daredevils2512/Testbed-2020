package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PIDDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.vision.Limelight;

public class Commands {
  
    public static Command pidDrive(PIDDrivetrain drivetrian, double move, double turn) {
      return new RunCommand(() -> drivetrian.drive(move, turn), drivetrian);
    }

    public static Command turnToAngle(PIDDrivetrain drivetrain, double targetAngle) {
      return new TurnToAngle(drivetrain, targetAngle);
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
        return new FindTarget(turret, limelight, tolerance);
    }
}