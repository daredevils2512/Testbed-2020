/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;

/**
 * Add your docs here.
 */
public interface ClosedLoopDrivetrain extends SimpleDrivetrain {
  double getMaxSpeed();
  double getMaxAngularSpeed();
  Pose2d getPose();
  void arcadeDriveClosedLoop(double velocity, double angularVelocity);
  void resetPose();
}
