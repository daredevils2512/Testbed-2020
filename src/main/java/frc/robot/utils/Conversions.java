/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

/**
 * Helper class for unit converions
 */
public final class Conversions {
    public static final int INCHES_PER_FOOT = 12;
    public static final double INCHES_PER_METER = 0.0254;
    public static final double METERS_PER_INCH = 39.3701;
    public static final double CENTIMETERS_PER_INCH = 2.54;
    public static final int CENTIMETERS_PER_METER = 100;
    public static final double FEET_PER_METER = 3.28084;
    public static final double METERS_PER_FOOT = 0.3048;

    private Conversions() {

    }

    public static double inchesToCentimeters(double inches) {
        return inches * CENTIMETERS_PER_INCH;
    }

    public static double feetToInches(double feet) {
        return feet * INCHES_PER_FOOT;
    }

    public static double feetToMeters(double feet) {
        return feet * METERS_PER_FOOT;
    }

    public static double feetAndInchesToInches(int feet, double inches) {
        return feetToInches(feet) + inches;
    }

    public static double feetAndInchesToMeters(int feet, double inches) {
        return feetToMeters(feet) + inchesToMeters(inches);
    }

    public static double inchesToMeters(double inches) {
        return inches * METERS_PER_INCH;
    }
}
