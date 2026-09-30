/**
 * Copyright 2025 ASAP Robotics (FTC Team 22029).
 * See LICENCE and NOTICE files for more details.
 */

package org.firstinspires.ftc.teamcode.types;

public class SystemReport {
  public final SystemStatus status;
  public final String message;

  public SystemReport(SystemStatus status, String message) {
    this.status = status;
    this.message = message;
  }

  public SystemReport(SystemStatus status) {
    this.status = status;
    switch (status) {
      case NOMINAL:
        this.message = "🟩Normal";
        break;

      case FALLBACK:
        this.message = "🟨Backup; performance may be degraded";
        break;

      case INOPERABLE:
        this.message = "🟥Broken";
        break;

      default:
        this.message = "⁉️Unknown (something is wrong)";
    }
  }
}