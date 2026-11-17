package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;

/**
 * Class to save the robot's position between OpModes, in a static member
 */
public class RobotPose {
  /**
   * The position of the robot between OpModes
   */
  public static Pose robotPose = new Pose(0.0, 0.0, 0.0);
}
