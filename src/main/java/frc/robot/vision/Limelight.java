package frc.robot.vision;

public enum Limelight {
  TURRET("limelight");

  private final String m_tableName;

  private Limelight(String tableName) {
    m_tableName = tableName;
  }

  public String getTableName() {
    return m_tableName;
  }
}