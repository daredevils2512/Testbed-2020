/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Limelight manager for power cell target tracking
 */
public final class LimelightUtil {
  public static final double MAX_HORIZONTAL_OFFSET = 29.8; // Angle in degrees
  public static final double MAX_VERTICAL_OFFSET = 24.85; // Angle in degrees

  private LimelightUtil() {
    
  }

  public static boolean hasTarget(final Pipeline pipeline) {
    usePipeline(pipeline);
    return getTable(pipeline).getEntry("tv").getBoolean(false);
  }

  public static double getHorizontalOffset(final Pipeline pipeline) {
    usePipeline(pipeline);
    return getTable(pipeline).getEntry("tx").getNumber(0).doubleValue();
  }

  public static double getVerticalOffset(final Pipeline pipeline) {
    usePipeline(pipeline);
    return getTable(pipeline).getEntry("ty").getNumber(0).doubleValue();
  }

  /**
   * Get how much the target contour fills the image
   * 
   * @param pipeline Limeligth pipeline to use
   * @return A decimal value between 0 and 1
   */
  public static double getFill(final Pipeline pipeline) {
    usePipeline(pipeline);
    return getTable(pipeline).getEntry("ta").getNumber(0).doubleValue();
  }

  public static double calculateDistance(final Pipeline pipeline, final double targetHeight, final double cameraAngle) {
    usePipeline(pipeline);
    return targetHeight / Math.tan(Math.toRadians(cameraAngle + getHorizontalOffset(pipeline)));
  }

  private static NetworkTable getTable(final Pipeline pipeline) {
    return NetworkTableInstance.getDefault().getTable(pipeline.getTableName());
  }

  private static void usePipeline(final Pipeline pipeline) {
    getTable(pipeline).getEntry("pipeline").setNumber(pipeline.getID());
  }

  /**
   * sets the pipeline to the default pipeline
   * @return distance in units of something to the tagret
   */
  // public double getDistanceToTarget() {
  //   setPipeline(m_defaultPipeline);
  //   return (m_defaultPipeline.getVericalOffset() / Math.tan(Math.toRadians(m_defaultPipeline.getAngle() + ty()))) - m_defaultPipeline.getHorizontalOffset();
  // }
}