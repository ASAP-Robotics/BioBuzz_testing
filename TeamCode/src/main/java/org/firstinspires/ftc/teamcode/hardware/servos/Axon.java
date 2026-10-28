/**
 * Copyright 2025-2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more
 * details.
 */
package org.firstinspires.ftc.teamcode.hardware.servos;

import static java.lang.Math.max;
import static java.lang.Math.min;
import static org.firstinspires.ftc.teamcode.constants.UpdateTolerance.SERVO_UPDATE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.utils.MathUtils.COMPARISON_THRESHOLD;
import static org.firstinspires.ftc.teamcode.utils.MathUtils.areEqual;
import static org.firstinspires.ftc.teamcode.utils.MathUtils.getDifferenceCircular;
import static org.firstinspires.ftc.teamcode.utils.MathUtils.map;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.interfaces.MonitoredSystem;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.types.SystemStatus;
import org.firstinspires.ftc.teamcode.utils.Follower;

/** Class to control Axon servos. Basically a wrapper around the `Servo` class to add encoder
 * feedback, rudimentary fault detection, and variable speed movement. */
public class Axon implements MonitoredSystem, System {
  public enum Type {
    /** Represents an Axon MAX+ */
    MAX(521.74), // 0.115 seconds / 60 degrees
    /** Represents an Axon MINI+ */
    MINI(666.67), // 0.090 seconds / 60 degrees
    /** Represents an Axon with an unknown or irrelevant type */
    UNKNOWN(500.0); // this speed should be irrelevant

    /** The no-load speed of the Axon at 6.0 volts, in degrees per second */
    public final double maxSpeed;

    /**
     * Constructor for Axon Types
     * @param maxSpeed the no-load speed of the Axon at 6.0 volts, in degrees per second
     */
    Type(double maxSpeed) {
      this.maxSpeed = maxSpeed;
    }
  }

  /** The type of the Axon (MAX+ or MINI+) */
  public final Type type;
  /** Backup simulation to model Axon movement if encoder fails */
  private final Follower movementSimulation;
  /** Used to slow Axon movement. <p>Units: degrees (per second)</p> */
  private final Follower targetFollower;
  /** The (Axon) servo being controlled */
  private final Servo servo;
  /** The encoder of the Axon being controlled */
  private final AnalogInput encoder;
  /** If true, the Axon won't use the encoder, and will always be "at target"*/
  private final boolean dummy;
  /** How far encoder readings can deviate from target and Axon still be "at target" */
  private final double toleranceDegrees;
  /** The operational status of the Axon */
  private SystemStatus status = SystemStatus.NOMINAL;
  /** The last measured position of the Axon from the encoder, in degrees */
  private double positionDegrees = 0.0;
  /** The last calculated value of if the Axon is "at target" */
  private boolean isAtTarget = false;

  /**
   * Creates a default Axon
   *
   * @param type the type of Axon this is (MAX+ or MINI+)
   * @param hardwareMap the hardware map
   * @param servoName the name of the servo
   * @param encoderName the name of the encoder
   */
  public Axon(Type type, HardwareMap hardwareMap, String servoName, String encoderName) {
    this(
        type,
        hardwareMap.get(Servo.class, servoName),
        hardwareMap.get(AnalogInput.class, encoderName)
    );
  }

  /**
   * Creates a dummy (no encoder) Axon with default parameters
   *
   * @param servo the servo to control
   */
  public Axon(Servo servo) {
    this(Type.UNKNOWN, servo, null, 5, true);
  }

  /**
   * Creates a default Axon
   *
   * @param type the type of Axon this is (MAX+ or MINI+)
   * @param servo the servo to control
   * @param encoder the encoder of the servo being controlled
   */
  public Axon(Type type, Servo servo, AnalogInput encoder) {
    this(type, servo, encoder, 5, false);
  }

  /**
   * Creates an Axon with advanced parameters exposed
   *
   * @param type the type of Axon this is (MAX+ or MINI+)
   * @param servo the servo to control
   * @param encoder the encoder of the servo being controlled
   * @param toleranceDegrees the amount the angle read can differ from the target angle and the
   *     servo still be considered "at target"
   * @param dummy if true, this Axon won't use its encoder
   * @throws IllegalArgumentException if any parameters are null, or if tolerance is not a number
   */
  public Axon(Type type, Servo servo, AnalogInput encoder, double toleranceDegrees, boolean dummy)
      throws IllegalArgumentException {
    if (type == null || servo == null || (encoder == null && !dummy) ||
        Double.isNaN(toleranceDegrees)) {
      throw new IllegalArgumentException();
    }
    this.type = type;
    this.servo = servo;
    this.encoder = encoder;
    this.dummy = dummy;
    this.toleranceDegrees = toleranceDegrees;
    // use half of maximum speed for conservative backup simulation
    this.movementSimulation =
        dummy ? null : new Follower(0.0, 0.0, COMPARISON_THRESHOLD, type.maxSpeed / 2);
    this.targetFollower = new Follower(0.0, 0.0, COMPARISON_THRESHOLD, Double.POSITIVE_INFINITY);
    if (dummy) this.isAtTarget = true;
  }

  @Override
  public void init() {
    if (!dummy) {
      double posDeg = readPositionDegrees();
      // hold current position on init
      targetFollower.setValue(posDeg);
      targetFollower.setTarget(posDeg);
      movementSimulation.setValue(posDeg);
      movementSimulation.setTarget(posDeg);
    }
    update();
  }

  @Override
  public void initLoop() {
    update();
  }

  @Override
  public void start() {
    update();
  }

  @Override
  public void loop() {
    update();
  }

  @Override
  public void stop() {
    update();
  }

  @Override
  public SystemReport getStatus() {
    if (status == SystemStatus.FALLBACK) {
      return new SystemReport(status, "🟨Encoder failure; performance will be degraded");
    } else {
      return new SystemReport(status);
    }
  }

  /**
   * Updates the Axon, its position, and its status
   */
  protected void update() {
    double lastSetPosition = servo.getPosition();
    double targetPositionDegrees = targetFollower.getValue();
    double targetSetPosition = targetPositionDegrees / 360;

    if (!areEqual(lastSetPosition, targetSetPosition, SERVO_UPDATE_TOLERANCE)) {
      servo.setPosition(targetSetPosition);
    }

    if (dummy) return;

    movementSimulation.setTarget(targetPositionDegrees);

    // read encoder position
    positionDegrees = readPositionDegrees();

    // update "at target" & operational status
    boolean encoderAtTarget =
        Math.abs(
            getDifferenceCircular(getPosition(), getTargetPosition(), 360.0, 0.0)
        ) <= toleranceDegrees;
    boolean followerAtTarget = movementSimulation.isAtTarget();
    // status being inconsistent is OK, it's primarily for driver info
    status = followerAtTarget && !encoderAtTarget ? SystemStatus.FALLBACK : SystemStatus.NOMINAL;
    // eventually being considered "at target" when jammed is intentional
    isAtTarget = encoderAtTarget || followerAtTarget;
  }

  /**
   * Gets the amount the angle read can differ from the target angle and the Axon still be
   * considered "at target"
   *
   * @return the tolerance, in degrees
   */
  public double getToleranceDegrees() {
    return toleranceDegrees;
  }

  /**
   * Sets the target position of the Axon, for the Axon to move to as fast as possible
   *
   * @param degrees the target position of the Axon, in degrees
   */
  public void setPosition(double degrees) {
    if (Double.isNaN(degrees) || Double.isInfinite(degrees) || degrees < 0.0 || degrees > 360.0) {
      throw new IllegalArgumentException("Position must be a finite number 0 to 360");
    }
    targetFollower.setTarget(degrees);
    targetFollower.setValue(degrees);
    update();
  }

  /**
   * Sets the target position of the Axon, for the Axon to move to at its configured speed
   *
   * @param degrees the target position of the Axon, in degrees
   * @param degreesPerSecond the target speed to move at, in degrees per second
   */
  public void setPositionSlow(double degrees, double degreesPerSecond) {
    setSpeed(degreesPerSecond);
    setPositionSlow(degrees);
  }

  /**
   * Sets the target position of the Axon, for the Axon to move to at its configured speed
   *
   * @param degrees the target position of the Axon, in degrees
   * @apiNote unless you've previously used setSpeed(), this will behave like setPosition()
   * @see setSpeed
   */
  public void setPositionSlow(double degrees) {
    if (Double.isNaN(degrees) || Double.isInfinite(degrees) || degrees < 0.0 || degrees > 360.0) {
      throw new IllegalArgumentException("Position must be a finite number 0 to 360");
    }
    targetFollower.setTarget(degrees);
    update();
  }

  /**
   * Gets the target position of the Axon
   *
   * @return the target position of the Axon, in degrees (0 if it hasn't been set yet)
   * @apiNote this method doesn't return the current position, it returns the <strong>target
   * position</strong>.
   */
  public double getTargetPosition() {
    return targetFollower.getTarget();
  }

  /**
   * Gets the last read position of the Axon
   *
   * @return the current position of the Axon, in degrees (from 0 to 360)
   * @apiNote if this Axon is a dummy this will always return 0
   */
  public double getPosition() {
    return dummy ? 0.0 : positionDegrees; // if dummy we can't get position
  }

  /**
   * Gets if the Axon is currently within tolerance of its target, or if it's been long enough that
   * it <i>should</i> be there (the Axon is jammed, or the encoder isn't working)
   *
   * @return true if the Axon is at its target, false if it isn't at its target
   * @apiNote if this Axon is a dummy this will always return true
   */
  public boolean atTarget() {
    return dummy || isAtTarget;
  }

  /**
   * Sets the target speed of the Axon, it will try to move approximately this fast.
   * @param degreesPerSecond the target speed of the Axon, in degrees per second
   * @throws IllegalArgumentException if speed is negative or not a number
   * @apiNote Set speed to infinity for maximum speed
   */
  public void setSpeed(double degreesPerSecond) {
    // Follower.setSpeed() already does all necessary value checks, and will throw exceptions
    targetFollower.setSpeed(degreesPerSecond);
  }

  /**
   * Gets the target speed of the Axon
   *
   * @return the target speed of the Axon, in degrees per second
   */
  public double getTargetSpeed() {
    return targetFollower.getSpeed();
  }

  /**
   * Reads the current position of the Axon in degrees (from 0 to 360)
   *
   * @return the current position of the axon, in degrees
   */
  private double readPositionDegrees() {
    // 0v = 0 degrees, 3.3v = 360 degrees
    // This is older code that worked, but I don't like how it handles dead banding of the encoder
    // Waiting until hardware testing to change ("don't fix what's not broken" mentality)
    return min(360, max(0, map(360 - ((encoder.getVoltage() / 3.3) * 360), 20, 340, 0, 360)));
  }
}
