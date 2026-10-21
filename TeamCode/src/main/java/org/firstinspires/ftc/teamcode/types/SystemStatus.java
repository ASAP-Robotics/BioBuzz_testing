/** Copyright 2025 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.types;

/** Simple enum to represent the operational status of robot systems */
public enum SystemStatus {
  UNKNOWN(2),
  NOMINAL(0),
  INACTIVE(0),
  FALLBACK(1),
  COMPROMISED(3),
  INOPERABLE(4);

  public final int severity;

  SystemStatus(int severity) {
    this.severity = severity;
  }
}
