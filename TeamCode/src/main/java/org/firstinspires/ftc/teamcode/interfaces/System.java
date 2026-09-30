/** Copyright 2025 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.interfaces;

import org.firstinspires.ftc.teamcode.types.SystemReport;

/**
 * Interface for systems that can report an operational status
 */
public interface System {
  /**
   * Gets the operational status of the system
   *
   * @return the status of the system
   */
  SystemReport getStatus();
}
