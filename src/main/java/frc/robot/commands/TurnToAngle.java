/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.PIDDrivetrain;

public class TurnToAngle extends PIDCommand {
  private final double m_turnTolerance = 5; // Degrees
  private final double m_turnSpeedTolerance = 1; // Degrees per second

  public TurnToAngle(PIDDrivetrain drivetrain, double targetAngle) {
    super(new PIDController(0.5, 0, 0), drivetrain::getYaw, targetAngle, output -> {
      drivetrain.drive(0, output * drivetrain.k_maxTurn);
    }, drivetrain);

    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(m_turnTolerance, m_turnSpeedTolerance);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
