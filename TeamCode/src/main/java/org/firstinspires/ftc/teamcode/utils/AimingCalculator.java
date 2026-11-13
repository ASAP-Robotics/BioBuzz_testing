/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Class to calculate aiming info from the position of the robot and a position to aim at
 */
public class AimingCalculator {
  private Pose target = new Pose(0.0, 0.0);
  private Pose position = new Pose(0.0, 0.0, 0.0);

  /**
   * Sets the position of the target used to calculate aiming info from
   *
   * @param target the target's current position
   */
  public void setTarget(Pose target) {
    this.target = target;
  }

  /**
   * Gets the position of the target used to calculate aiming info
   *
   * @return the position of the target
   */
  public Pose getTarget() {
    return target;
  }

  /**
   * Sets the position used to calculate aiming info from
   *
   * @param position the robot's current position
   */
  public void setPosition(Pose position) {
    this.position = position;
  }

  /**
   * Gets the position use to calculate aiming info from
   *
   * @return the last set position of the robot
   */
  public Pose getPosition() {
    return position;
  }

  /**
   * Calculates the distance to the target
   *
   * @return the distance to the target, in PedroPathing units (inches??? IDK)
   */
  public double getDistance() {
    return target.distance(position);
  }

  /**
   * Gets the heading of the target relative to the robot (0 = X+, 90 = Y+, 180 = X-, 270 = Y-)
   *
   * @return the heading of the target from the robot, in degrees
   */
  public double getHeading() {
    double distX = target.x() - position.x(); // X distance (b)
    double distY = target.y() - position.y(); // Y distance (a)
    double angleA = Math.atan2(distY, distX); // angle A, referencing X-axis (tan A = a/b) (radians)
    angleA = AngleUnit.DEGREES.fromRadians(angleA); // convert to degrees
    return AngleUnit.normalizeDegrees(angleA); // normalize from 0-360
  }
}
