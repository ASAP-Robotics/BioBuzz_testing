package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.commands.SetFlowerRamp;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.FlowerRamp;

public class AutoPathsRed extends AutoPaths {
  public static final Pose start = poseFactory.of(56, 9, 90);

  private static final Pose shoot = poseFactory.of(56, 30, 90);
  private static final Pose inFrontOfFlower = poseFactory.of(18, 47, 180);
  private static final Pose shoot_flower_control_1 = poseFactory.of(36, 28, 0);
  private static final Pose shoot_flower_control_2 = poseFactory.of(32, 48, 0);
  private static final Pose intakeFromFlower = poseFactory.of(12, 47, 180);
  private static final Pose park = poseFactory.of(12, 90, 90);
  private static final Pose driveToParkControl1 = poseFactory.of(18, 28, 0);

  public static Command shoot() {
    return new Shoot(robot);
  }

  public static Command dropRamp() {
    return new SetFlowerRamp(robot.flowerRamp, FlowerRamp.State.DEPLOYED);
  }

  public static Command raiseRamp() {
    return new SetFlowerRamp(robot.flowerRamp, FlowerRamp.State.STOWED);
  }

  public static Path driveToShoot1() {
    return Paths.line(start, shoot).linear(start, shoot);
  }

  public static Path driveToEmptyFlower() {

    return Paths.path(Paths.curve(shoot, shoot_flower_control_1, shoot_flower_control_2, inFrontOfFlower).linear(shoot, inFrontOfFlower), Paths.line(inFrontOfFlower, intakeFromFlower).constant(intakeFromFlower));
  }

  public static Path driveToShoot2() {
    return Paths.path(Paths.line(intakeFromFlower, inFrontOfFlower).constant(inFrontOfFlower), Paths.curve(inFrontOfFlower, shoot_flower_control_2, shoot_flower_control_1, shoot).linear(inFrontOfFlower, shoot));
  }

  public static Path driveToPark() {
    return Paths.curve(shoot, driveToParkControl1, park).constant(park);
  }

  public static Command RED_CLOSE_AUTO =
      sequential(
          follow(follower, driveToShoot1()),
          parallel(
              shoot(),
              dropRamp()
          ),
          follow(follower, driveToEmptyFlower()),
          follow(follower, driveToShoot2()),
          parallel(
              shoot(),
              raiseRamp()
          ),
          follow(follower, driveToPark())
      ); // todo
}
