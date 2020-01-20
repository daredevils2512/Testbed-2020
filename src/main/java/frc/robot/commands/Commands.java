package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Drivetrain;

public class Commands {
    public static Command arcadeDrive(Drivetrain drivetrain, DoubleSupplier moveSupplier, DoubleSupplier turnSupplier) {
        return new RunCommand(() -> drivetrain.arcadeDrive(moveSupplier.getAsDouble(), turnSupplier.getAsDouble()), drivetrain);
    }
}