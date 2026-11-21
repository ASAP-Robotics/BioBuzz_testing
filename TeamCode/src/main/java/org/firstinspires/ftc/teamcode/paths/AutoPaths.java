/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.commands.SetBasket;
import org.firstinspires.ftc.teamcode.commands.SetFlowerRamp;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.configurations.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.FlowerRamp;
import org.firstinspires.ftc.teamcode.systems.ScoringSystem;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.OpModeInfo;

public class AutoPaths {
  private final RobotHardware robot;

  private final ScoringSystem.Basket basket;

  private final Pose start;
  private final Pose shoot;
  private final Pose inFrontOfFlower;
  private final Pose shoot_flower_control_1;
  private final Pose shoot_flower_control_2;
  private final Pose intakeFromFlower;
  private final Pose park;
  private final Pose driveToParkControl1;

  public AutoPaths(RobotHardware r) {
    if (r == null) {
      throw new IllegalArgumentException("Robot hardware must not be null");
    }

    // rotate 180 degrees around center of field if Blue
    PoseFactory p =
        OpModeInfo.color == AllianceColor.RED
            ? PoseFactory.degrees()
            : PoseFactory.degrees().rotateAround(70.75, 70.75, 180.0);

    this.basket =
        OpModeInfo.color == AllianceColor.RED ?
            ScoringSystem.Basket.CLOSE : ScoringSystem.Basket.FAR;

    this.robot = r;

    // ALL PATH POINTS HERE ARE FOR RED; blue paths are created from them
    this.start = p.of(56.25, 9, 90);
    this.shoot = p.of(56, 30, 90);
    this.inFrontOfFlower = p.of(18, 47, 180);
    this.shoot_flower_control_1 = p.of(36, 28, 0);
    this.shoot_flower_control_2 = p.of(32, 48, 0);
    this.intakeFromFlower = p.of(14, 47, 180);
    this.park = p.of(12, 90, 90);
    this.driveToParkControl1 = p.of(18, 28, 0);
  }

  protected Command setBasket() {
    // could be updated to support flipping, but fine for now
    return new SetBasket(robot, basket);
  }

  protected Command shoot() {
    return new Shoot(robot);
  }

  protected Command dropRamp() {
    return new SetFlowerRamp(robot.flowerRamp, FlowerRamp.State.DEPLOYED);
  }

  protected Command raiseRamp() {
    return new SetFlowerRamp(robot.flowerRamp, FlowerRamp.State.STOWED);
  }

  protected Path driveToShoot1() {
    return Paths.line(start, shoot).linear(start, shoot);
  }

  protected Path driveToEmptyFlower1() {
    return Paths.curve(shoot, shoot_flower_control_1, shoot_flower_control_2, inFrontOfFlower).linear(shoot, inFrontOfFlower);
  }
  protected Path driveToEmptyFlower2() {
    return Paths.line(inFrontOfFlower, intakeFromFlower).constant(intakeFromFlower);
  }

  protected Path driveToShoot2_1() {
    return Paths.line(intakeFromFlower, inFrontOfFlower).constant(inFrontOfFlower);
  }

  protected Path driveToShoot2_2() {
    return Paths.curve(inFrontOfFlower, shoot_flower_control_2, shoot_flower_control_1, shoot).linear(inFrontOfFlower, shoot);
  }

  protected Path driveToPark() {
    return Paths.curve(shoot, driveToParkControl1, park).constant(park);
  }

  /**
   * Gets the robot's starting position for the "close" Auto
   *
   * @return the starting point of the "close" auto
   */
  public Pose getCloseAutoStart() {
    return start;
  }

  /**
   * Gets a Command containing the pathing and controls for the "close" Auto
   *
   * @return the "Close" auto
   */
  public Command getCloseAuto() {
    return sequential(
        raiseRamp(),
        parallel(follow(robot.follower, driveToShoot1()), setBasket()),
        shoot(),
        dropRamp(),
        follow(robot.follower, driveToEmptyFlower1()),
        follow(robot.follower, driveToEmptyFlower2()),
        Commands.waitMs(5000),
        follow(robot.follower, driveToShoot2_1()),
        follow(robot.follower, driveToShoot2_2()),
        raiseRamp(),
        shoot(),
        follow(robot.follower, driveToPark()));
  }
}
