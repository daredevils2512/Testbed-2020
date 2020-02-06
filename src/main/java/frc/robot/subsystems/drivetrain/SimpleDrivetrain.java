/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Add your docs here.
 */
public interface SimpleDrivetrain extends Subsystem {
  default void arcadeDrive(double move, double turn) {
    driveLeft(move + turn);
    driveRight(move - turn);
  }
  
  void driveLeft(double speed);
  void driveRight(double speed);
}
