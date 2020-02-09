package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PiTable {

    private NetworkTable m_table;
    private double m_maxRange;

    public PiTable() {
        m_table = NetworkTableInstance.getDefault().getTable("ball table");
    }

    public boolean hasTarget() {
        return m_table.getSubTable("info").getEntry("has target").getBoolean(false);
    }

    public int getNumberOfTargets() {
        return (int)m_table.getSubTable("info").getEntry("targets").getDouble(0.0);
    }

    public double[] getClosestTarget() {
        double dist = m_maxRange;
        int i = 1;
        for (; i <= getNumberOfTargets(); i++) {
            dist = dist >= m_table.getEntry("ball " + i).getDoubleArray(new double[]{0.0})[2] ? 
                dist : m_table.getEntry("ball" + i).getDoubleArray(new double[]{0.0})[2];
        }
        i -= 1;
        return m_table.getEntry("ball " + i).getDoubleArray(new double[]{0.0});
    }
}