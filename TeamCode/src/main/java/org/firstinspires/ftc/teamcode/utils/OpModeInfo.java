/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.OpModeType;

/**
 * Class to hold definitive source of info about the currently running OpMode. Info in this class
 * should be set first thing by the active OpMode on init, and shouldn't be changed by other code
 */
public class OpModeInfo {
  public static OpModeType type = OpModeType.TELEOP;
  public static AllianceColor color = AllianceColor.RED;
}
