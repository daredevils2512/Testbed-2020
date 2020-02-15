/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.utils.Conversions;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double hexagonCenterHeight = Conversions.feetAndInchesToInches(8, 2.25); // Height in inches

  // TODO: Check target position/height
  // At (0, 0) facing backwards
  public static final Pose2d TARGET_POSE_FIELD_RELATIVE = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180));
  public static final double TARGET_CENTER_HEIGHT = Conversions.feetAndInchesToMeters(8, 2.25);
}
