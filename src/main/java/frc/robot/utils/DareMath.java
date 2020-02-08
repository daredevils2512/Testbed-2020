package frc.robot.utils;

public final class DareMath {
  private DareMath() {

  }

  public static double mapRange(double value, double oldMin, double oldMax, double newMin, double newMax) {
    double rangeTemp = (value - oldMin) / (oldMax - oldMin);
    return rangeTemp * (newMax - newMin) + newMin;
  }
}