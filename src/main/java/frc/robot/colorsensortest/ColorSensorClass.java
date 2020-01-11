package frc.robot.colorsensortest;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensorClass{
  public enum SpinnerColor {
    Red,
    Green,
    Blue,
    Yellow,
    Unknown;
  }

  private final ColorSensorV3 colorSensor;

  private final ColorMatch colorMatch = new ColorMatch();
  private final Color m_matchRed = ColorMatch.makeColor(0.55, 0.325, 0.125);
  private final Color m_matchGreen = ColorMatch.makeColor(0.15, 0.6, 0.25);
  private final Color m_matchBlue = ColorMatch.makeColor(0.1, 0.4, 0.5);
  private final Color m_matchYellow = ColorMatch.makeColor(0.3, 0.6, 0.1);

  public ColorSensorClass() {
    colorSensor = new ColorSensorV3(Port.kOnboard);

    colorMatch.addColorMatch(m_matchRed);
    colorMatch.addColorMatch(m_matchGreen);
    colorMatch.addColorMatch(m_matchBlue);
    colorMatch.addColorMatch(m_matchYellow);
  }

  // blue is slightly harder to see at max distance
  public SpinnerColor getColorMatch() {
    Color colorData = colorSensor.getColor();
    ColorMatchResult matchResult = colorMatch.matchClosestColor(colorData);
    if (this.getDistance() >= 10) {
      return SpinnerColor.Unknown;
    } else if (matchResult.color == m_matchRed) {
      return SpinnerColor.Red;
    } else if (matchResult.color == m_matchGreen) {
      return SpinnerColor.Green;
    } else if (matchResult.color == m_matchBlue) {
      return SpinnerColor.Blue;
    } else if (matchResult.color == m_matchYellow) {
      return SpinnerColor.Yellow;
    } else {
      return SpinnerColor.Unknown;
    }
  }

  public Color getColor() {
    return colorSensor.getColor();
  }

  public int getProximity() {
    return colorSensor.getProximity();
  }

    // link to graph https://www.desmos.com/calculator/5003n6o27s
    // thank jared for the graph
    // distance in inches
    public double getDistance() {
      double currentProx = this.getProximity();
      double a = 45.2291170447;
      double b = 0.981074042624;
      double c = 0.947344282838;
      return ((a * (Math.pow(b, currentProx))) + c);
  }
}