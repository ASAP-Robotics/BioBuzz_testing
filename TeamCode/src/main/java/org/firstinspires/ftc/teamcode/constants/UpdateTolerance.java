/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class UpdateTolerance {
  /** The max allowable change in a servo target position before a new value should be sent */
  public static double SERVO_UPDATE_TOLERANCE =
      0.005; // approximately 1 degree for 180 degree servo

  /** The max allowable change in a servo target position before a new value should be sent */
  public static double SERVO_UPDATE_TOLERANCE_DEGREES = 1.0;

  /** The max allowable change in a motor target power before a new value should be sent */
  public static double MOTOR_UPDATE_TOLERANCE = 0.01;
}
