package frc.robot.vision;

/**
 * Identical pipelines on different limelights should each have
 * their own enum value
 */
public enum Pipeline {
  POWER_CELLS(Limelight.TURRET, 0),
  POWER_CELLS_GRIP(Limelight.TURRET, 1),
  POWER_CELL_TARGET(Limelight.TURRET, 3);

  private final Limelight m_limelight;
  private final int m_id;

  private Pipeline(Limelight limelight, int id) {
    m_limelight = limelight;
    m_id = id;
  }

  public Limelight getLimelight() {
    return m_limelight;
  }

  public int getID() {
    return m_id;
  }

  public String getTableName() {
    return m_limelight.getTableName();
  }
}