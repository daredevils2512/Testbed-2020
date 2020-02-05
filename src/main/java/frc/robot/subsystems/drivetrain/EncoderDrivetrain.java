/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain;

public interface EncoderDrivetrain extends SimpleDrivetrain {
  double getLeftDistance();
  double getRightDistance();
  double getLeftVelocity();
  double getRightVelocity();
  void resetDriveEncoders();
}
