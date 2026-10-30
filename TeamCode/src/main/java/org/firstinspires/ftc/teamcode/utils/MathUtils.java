/**
 * Copyright 2025-2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more
 * details.
 */
package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Class containing miscellaneous math utilities */
public class MathUtils {
  public static final double COMPARISON_THRESHOLD = 1e-6;

  /**
   * Maps a number from one range to another
   *
   * @param x the number to map
   * @param inMin the minimum of the input range
   * @param inMax the maximum of the input range
   * @param outMin the minimum of the output range
   * @param outMax the maximum of the output range
   * @return the mapped number
   */
  public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
  }

  /**
   * Clamps a given value between a min and max
   *
   * @param val the value to clamp
   * @param min the minimum value
   * @param max the maximum value
   * @return the clamped value
   */
  public static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }

  /**
   * Finds the closest value to `input` that is equal to `desired` modulo `offset`
   *
   * @param input the initial, unadjusted input value
   * @param desired the value we want to be close to
   * @param offset the size of offset steps
   * @return the optimized value
   */
  public static double closestWithOffset(double input, double desired, double offset) {
    double numOffsets = Math.round((desired - input) / offset);
    return input + numOffsets * offset;
  }

  /**
   * Normalizes an angle around a center point
   *
   * @param angle the angle to normalize
   * @param center the angle to normalize around
   * @return the normalized angle
   * @apiNote units are in degrees
   */
  public static double normalizeAround(double angle, double center) {
    return AngleUnit.normalizeDegrees(angle - center) + center;
  }

  /**
   * Gets the difference between two positions
   *
   * @param pose1 the first position
   * @param pose2 the second position
   * @return the difference between the first and second positions
   */
  public static Pose2D poseDifference(Pose2D pose1, Pose2D pose2) {
    return new Pose2D(
        DistanceUnit.INCH,
        pose1.getX(DistanceUnit.INCH) - pose2.getX(DistanceUnit.INCH),
        pose1.getY(DistanceUnit.INCH) - pose2.getY(DistanceUnit.INCH),
        AngleUnit.DEGREES,
        AngleUnit.normalizeDegrees(
            pose1.getHeading(AngleUnit.DEGREES) - pose2.getHeading(AngleUnit.DEGREES)));
  }

  /**
   * Gets if two numbers are effectively the same
   *
   * @param a the first number
   * @param b the second number
   * @param tolerance the tolerance to use when comparing numbers
   * @return if the numbers are within the tolerance of each other
   */
  public static boolean areEqual(double a, double b, double tolerance) {
    return Math.abs(a - b) <= tolerance;
  }

  /**
   * Gets if two numbers are effectively the same
   *
   * @param a the first number
   * @param b the second number
   * @return if the numbers are close enough to be "equal"
   */
  public static boolean areEqual(double a, double b) {
    return areEqual(a, b, COMPARISON_THRESHOLD);
  }

  /**
   * Gets the difference between two values in a circular number space (e.g. where 0 == 360)
   *
   * @param a the first number
   * @param b the second number
   * @param max the maximum (inclusive) value of the circular space
   * @param min the minimum (inclusive) value of the circular space
   * @return the difference between the two values
   */
  public static double getDifferenceCircular(double a, double b, double max, double min) {
    if (Double.isNaN(a)
        || Double.isNaN(b)
        || Double.isNaN(max)
        || Double.isNaN(min)
        || min >= max
        || a > max
        || a < min
        || b > max
        || b < min) {
      throw new IllegalArgumentException();
    }

    double range = max - min;
    double diff = (a - b) % range;
    if (diff < -range / 2.0) {
      diff += range;
    } else if (diff > range / 2.0) {
      diff -= range;
    }
    return diff;
  }

  /**
   * Converts an angular velocity from angle units per second to revolutions per minute (RPM)
   *
   * @param unit the unit of the provided angular velocity
   * @param unitsPerSecond the value of the angular velocity to be converted, in units per second
   * @return the provided angular velocity, in revolutions per minute (RPM)
   */
  public static double angleUnitsPerSecondToRpm(AngleUnit unit, double unitsPerSecond) {
    return AngleUnit.DEGREES.fromUnit(unit, unitsPerSecond) / 6.0; // 6 = 360 / 60
  }

  /**
   * Converts an angular velocity from revolutions per minute (RPM) to angle units per second
   *
   * @param unit the angle unit for the returned angular velocity
   * @param rpm the angular velocity to convert from, in revolutions per second (RPM)
   * @return the provided angular velocity, in angle units per second
   */
  public static double rpmToAngleUnitsPerSecond(AngleUnit unit, double rpm) {
    return unit.fromDegrees(rpm * 6.0); // 6 = 360 / 60
  }
}
