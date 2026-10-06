/** Copyright 2025 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

/** Simple class to model a system where a value moves to a target linearly over time */
public class Follower {
  protected double target; // the target position of the modeled system
  protected double value; // the current position of the modeled system
  protected final double tolerance; // the tolerance of the modeled system
  protected final double unitsPerSecond; // the speed of the modeled system, in units per second
  protected ElapsedTime updateTime; // the time since the last update

  /**
   * Makes a new Follower
   *
   * @param value the initial value of the modeled system
   * @param target the initial target of the modeled system
   * @param tolerance the amount the value can differ from the target and be "at target"
   * @param unitsPerSecond the speed at which the modeled system moves, in units per second
   */
  public Follower(double value, double target, double tolerance, double unitsPerSecond) {
    this.value = value;
    this.target = target;
    this.tolerance = tolerance;
    this.unitsPerSecond = unitsPerSecond;
    this.updateTime = new ElapsedTime();
  }

  /**
   * Gets the tolerance of the modeled system
   *
   * @return the tolerance of the system
   */
  public double getTolerance() {
    return tolerance;
  }

  /**
   * Sets the target value of the modeled system
   *
   * @param target the target value of the modeled system
   */
  public void setTarget(double target) {
    this.target = target;
  }

  /**
   * Gets the target value of the modeled system
   *
   * @return the target value of the system
   */
  public double getTarget() {
    return target;
  }

  /**
   * Gets if the modeled system is at its target value
   *
   * @return true if at target, false otherwise
   */
  public boolean isAtTarget() {
    return Math.abs(getTarget() - getValue()) <= getTolerance();
  }

  /**
   * Overrides the value of the modeled system
   *
   * @apiNote this is an advanced feature, and usually isn't needed. use with discretion
   * @param value the new value of the modeled system
   */
  public void setValue(double value) {
    this.value = value;
  }

  /**
   * Gets the current value of the system
   *
   * @return the current value of the system
   */
  public double getValue() {
    update();
    return value;
  }

  /** Updates the current value of the modeled system */
  protected void update() {
    double deltaTime = updateTime.seconds();
    updateTime.reset();
    double error = target - value;
    double maxChange = unitsPerSecond * deltaTime;
    value += Math.copySign(Math.min(Math.abs(error), maxChange), error);
  }
}
