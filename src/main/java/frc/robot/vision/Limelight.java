package frc.robot.vision;

public enum Limelight {
  TURRET("limelight", 30); // TODO: Update turret limelight angle of elevation

  private final String m_tableName;
  private final double m_angleOfElevation;

  /**
   * Create a new Limelight
   * @param tableName Name of corresponding network table
   * @param angleOfElevation Angle from horizontal in degrees
   */
  private Limelight(String tableName, double angleOfElevation) {
    m_tableName = tableName;
    m_angleOfElevation = angleOfElevation;
  }

  public String getTableName() {
    return m_tableName;
  }

  public double getAngleOfElevation() {
    return m_angleOfElevation;
  }
}