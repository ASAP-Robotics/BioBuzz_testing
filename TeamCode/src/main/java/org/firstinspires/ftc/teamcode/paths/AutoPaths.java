package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.configurations.RobotHardware;

public abstract class AutoPaths {
  protected static Follower follower;
  protected static RobotHardware robot;
  protected static final PoseFactory poseFactory = PoseFactory.degrees();

  public static void setFollower(Follower newFollower) {
    follower = newFollower;
  }

  public static void setRobot(RobotHardware robot) {
    AutoPathsRed.robot = robot;
  }
}
