/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.utils;

/** Simple class to provide authoritative timestamping across OpModes */
public class Timestamp {
  /**
   * Gets the current time according to the system clock
   *
   * @return the current time, in milliseconds
   * @implNote we use milliseconds instead of nanoseconds because they are more intuitive
   */
  public static long now() {
    return System.nanoTime() / 1000;
  }
}
