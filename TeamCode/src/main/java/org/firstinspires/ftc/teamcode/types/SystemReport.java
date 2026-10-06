/** Copyright 2025 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.types;

/** Simple class to standardize status reports from robot systems */
public class SystemReport {
  public final SystemStatus status;
  public final String message;

  /**
   * Generates a system status report from a status and a message
   *
   * @param status the status of the system
   * @param message a string of text to be displayed to the driver
   */
  public SystemReport(SystemStatus status, String message) {
    this.status = status;
    this.message = message;
  }

  /**
   * Generates a system status report from a status
   *
   * @param status the status of the system
   * @apiNote this generates a generic message from the status
   */
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
