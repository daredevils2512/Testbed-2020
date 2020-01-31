/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.utils.Conversions;

/**
 * Limelight manager for power cell target tracking
 */
public class Limelight {
  public enum Pipeline {
    //table is the table name without the limelight- infromt of it. this also means which physical limelight it is
    PowerCellsLimelight(1, "balls"),
    PowerCells(0, "balls"),
    HexagonThing3d(2, "balls", 21, 7.34, 0); //inches. pipeline id is for testing on the old limelight

    private int m_id;
    private String m_table;
    private double m_angle;
    private double m_verticalOffset;
    private double m_horizontalOffset;

    private Pipeline(int id, String table) {
      m_id = id;
      m_table = table;
    }

    private Pipeline(int id, String table, double angle, double verticalOffset, double horizontalOffset) {
      m_id = id;
      m_table = "limelight-" + table;
      m_angle = angle;
      m_verticalOffset = Constants.hexagonCenterHeight - verticalOffset;
      m_horizontalOffset = horizontalOffset;
    }

    public String getTable() {
      return m_table;
    }

    public int getID() {
      return m_id;
    }

    public double getAngle() {
      return m_angle;
    }

    public double getVericalOffset() {
      return m_verticalOffset;
    }

    public double getHorizontalOffset() {
      return m_horizontalOffset;
    }
  }

  private NetworkTable m_table;

  private double lastPostion;
  private Pipeline m_defaultPipeline;

  public Limelight(Pipeline defaultPipeline) {
    m_defaultPipeline = defaultPipeline;
    m_table = NetworkTableInstance.getDefault().getTable(m_defaultPipeline.getTable());
    this.lastPostion = 1.0;
  }

  //Limelight table getters

  public void setPipeline(Pipeline pipeline) {
    m_table.getEntry("pipeline").setNumber(pipeline.getID());
  }

  public Pipeline getDefualtPipeline() {
    return m_defaultPipeline;
  }

  public boolean hasTarget() {
    return m_table.getEntry("tv").getDouble(0) == 1;
  }

  public double tx() {
    return m_table.getEntry("tx").getDouble(0);
  }

  public double ty() {
    return m_table.getEntry("ty").getDouble(0);
  }

  public double ta() {
    return m_table.getEntry("ta").getDouble(0);
  }

  public double ts() {
    return m_table.getEntry("ts").getDouble(0);
  }

  public double tl() {
    return m_table.getEntry("tl").getDouble(0);
  }

  public double getLastPosition() {
    if (tx() != 0) { 
      lastPostion = tx(); 
    }
    return lastPostion;
  }

  /**
   * sets the pipeline to the default pipeline
   * @return distance in units of something to the tagret
   */
  public double getDistanceToTarget() {
    setPipeline(m_defaultPipeline);
    return (m_defaultPipeline.getVericalOffset() / Math.tan(Math.toRadians(m_defaultPipeline.getAngle() + ty()))) - m_defaultPipeline.getHorizontalOffset();
  }
}