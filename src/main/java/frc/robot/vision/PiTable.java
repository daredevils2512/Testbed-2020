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

    /**
     * x offset in degrees
     * <p> really just entry[0] but like this for verbose or somehting
     * @param entry a double array returned by one of the getters
     * @return the index zero of the entry
     */
    public double getXOffset(double[] entry) {
        return entry[0];
    }
    
    /**
     * y offset in degrees
     * <p> really just entry[1] but like this for verbose or somehting
     * @param entry a double array returned by one of the getters
     * @return the index one of the entry
     */
    public double getYOffset(double[] entry) {
        return entry[1];
    }

    /**
     * distance in meters
     * <p> really just entry[2] but like this for verbose or somehting
     * @param entry a double array returned by one of the getters
     * @return the index two of the entry
     */
    public double getDistance(double[] entry) {
        return entry[2];
    }

    /**
     * size of target in pixels 
     * <p> not too useful but its one of the things returned by the pipeline so why not
     * <p> really just entry[3] but like this for verbose or somehting
     * @param entry a double array returned by one of the getters
     * @return the index three of the entry
     */
    public double getSize(double[] entry) {
        return entry[3];
    }

    /**
     * gets closest target
     * @return double array {0.0, 0.0, 0.0, 0.0} if no target found or if the tables screwed up
     */
    public double[] getClosestTarget() {
        if (m_table.getSubTable("info").getEntry("closest target").getDouble(0.0) != 0) {
            return m_table.getEntry(m_table.getSubTable("info").getEntry("closest target").getString("ball " + 1)).getDoubleArray(new double[]{0.0, 0.0, 0.0, 0.0});
        } else {
            return new double[]{0.0, 0.0, 0.0, 0.0};
        }
    }

    public double[] getBall(int ball) {
        return m_table.getEntry("ball " + ball).getDoubleArray(new double[]{0.0, 0.0, 0.0, 0.0});
    }

    public double[] getLastClosestTarget() {
        double[] lastClosestTargetPosition = new double[]{0.0, 0.0, 0.0, 0.0};
        if (hasTarget()) {
            lastClosestTargetPosition = getClosestTarget();
            return lastClosestTargetPosition;
        } else {
            return lastClosestTargetPosition;
        }
    }
}