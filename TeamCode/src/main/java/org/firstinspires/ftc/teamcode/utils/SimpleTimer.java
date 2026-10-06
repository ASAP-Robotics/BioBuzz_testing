/**
 * Copyright 2025-2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more
 * details.
 */
package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Class to model a simple timer
 *
 * @apiNote use judiciously, often using raw ElapsedTime objects can be better
 */
public class SimpleTimer {
  private final ElapsedTime timer = new ElapsedTime();
  private double duration; // seconds
  private boolean running = false;

  public SimpleTimer() {
    this(1.0);
  }

  public SimpleTimer(double durationSeconds) {
    this.duration = durationSeconds;
  }

  /** Start or restart the timer */
  public void start() {
    timer.reset();
    running = true;
  }

  /** Start or restart the timer with a set duration */
  public void start(double durationSeconds) {
    this.duration = durationSeconds;
    start();
  }

  /** Stop the timer */
  public void stop() {
    running = false;
  }

  /** Call each loop to update the timer */
  public void update() {
    if (running && timer.seconds() >= duration) {
      running = false; // auto-stop when done
    }
  }

  /**
   * Returns if the timer is running
   *
   * @return true if the timer is running, false if the timer isn't running
   */
  public boolean isRunning() {
    update();
    return running;
  }

  /**
   * Returns if the timer is done
   *
   * @return true if the timer is done, false if the timer isn't done
   */
  public boolean isFinished() {
    update();
    return !running && timer.seconds() >= duration;
  }

  /**
   * Returns time since the timer was started
   *
   * @return the elapsed time since the timer was started
   */
  public double elapsed() {
    update();
    return timer.seconds();
  }

  /**
   * Returns the time left on the timer
   *
   * @return the amount of time left on the timer, or 0 if finished
   */
  public double remaining() {
    update();
    return Math.max(0, duration - timer.seconds());
  }
}
