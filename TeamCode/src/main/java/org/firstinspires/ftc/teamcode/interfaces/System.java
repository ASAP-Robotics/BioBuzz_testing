/**
 * Copyright 2025 ASAP Robotics (FTC Team 22029).
 * See LICENCE and NOTICE files for more details.
 */

package org.firstinspires.ftc.teamcode.interfaces;

import org.firstinspires.ftc.teamcode.types.SystemReport;

/**
 * @brief interface for systems that can report a status
 */
public interface System {
  /**
   * @brief gets the operational status of the system
   * @return the status of the system
   */
  SystemReport getStatus();
}