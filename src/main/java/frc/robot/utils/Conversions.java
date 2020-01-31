/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

/**
 * Add your docs here.
 */
public final class Conversions {

    public static final int inchesPerFoot = 12;
    public static final double cmPerInch = 2.54;
    public static final double cmPerMeter = 100;

    private Conversions() {
    }

    public static double inchesTocm(double inches) {
        return inches * cmPerInch;
    }

    public static double feetInchesToInches(int feet, double inches) {
        return (feet * inchesPerFoot) + inches;
    }

    public static double inchesToMeters(double inches) {
        return inchesTocm(inches) / cmPerMeter;
    }
}
