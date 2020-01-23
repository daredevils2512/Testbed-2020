package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PIDdrivetrain;

public class Commands {
    public static Command arcadeDrive(Drivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
      return new RunCommand(() -> drivetrain.arcadeDrive(moveSupplier.getAsDouble(), turnSupplier.getAsDouble()), drivetrain);
    }

    public static Command driveinches(PIDdrivetrain drivetrain, double inches) {
      return new InstantCommand(() -> drivetrain.driveDistance(inches), drivetrain);
    }
}