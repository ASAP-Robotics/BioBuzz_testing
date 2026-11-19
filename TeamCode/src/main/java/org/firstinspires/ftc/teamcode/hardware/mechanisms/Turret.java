/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import static org.firstinspires.ftc.teamcode.constants.Tolerances.MOTOR_UPDATE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.utils.MathUtils.areEqual;
import static org.firstinspires.ftc.teamcode.utils.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.utils.MathUtils.normalizeAround;
import static org.firstinspires.ftc.teamcode.utils.MathUtils.ticksToAngleUnits;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.sensors.BreakBeam;
import org.firstinspires.ftc.teamcode.interfaces.MonitoredSystem;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.types.OpModeType;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.types.SystemStatus;
import org.firstinspires.ftc.teamcode.utils.Follower;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.OpModeInfo;
import org.firstinspires.ftc.teamcode.utils.TimeWindowAverage;

@Configurable
public class Turret implements MonitoredSystem, System {
  /** Simple enum to represent operating state of turret */
  public enum State {
    /** Turret is disabled, and no motor power is applied */
    DISABLED,
    /** Turret is tracking a target position */
    NORMAL,
    /** Turret is recalibrating zero point */
    HOMING
  }

  /** Enum to keep track of the state while homing */
  protected enum HomingState {
    /** Turret is moving forwards from its position when homing was started */
    FORWARD(180.0),
    /** Turret is moving back past its position when homing was started */
    BACK(-180.0);

    public final double target;

    HomingState(double target) {
      this.target = target;
    }
  }

  // **Constant vals********************************************************************************
  private static final int MOTOR_GEAR_TEETH = 24;

  private static final double TURRET_GEAR_TEETH = 83;

  private static final double MOTOR_TICKS_PER_REV = (1.0 + (46.0 / 11.0)) * 28;

  /** How many encoder ticks are in one full rotation of the turret (not the motor) */
  private static final double TICKS_PER_REV =
      MOTOR_TICKS_PER_REV * TURRET_GEAR_TEETH / MOTOR_GEAR_TEETH;

  // **Tunable vals*********************************************************************************
  /** If telemetry from turret should be updated each loop */
  public static boolean TELEMETRY_ENABLE = false;

  /** The speed below which, if the turret moves at it, the turret will be considered stuck */
  public static double POSITION_SIM_DEGREES_PER_SECOND = 100.0; // todo tune

  /** The speed that the turret will attempt to move at when going to a set target */
  public static double NORMAL_DEGREES_PER_SECOND = Double.POSITIVE_INFINITY;

  /** The speed the turret will attempt to move at when homing */
  public static double HOMING_DEGREES_PER_SECOND = 30.0; // todo tune

  /** Roughly how close the turret can be to its target position and be "at target" */
  public static double TOLERANCE_DEGREES = 2.0;

  /** The physical minimum position the turret can move to (0 = straight forward) */
  public static double MIN_POSITION_DEGREES = -180.0;

  /** The physical maximum position the turret can move to (0 = straight forward) */
  public static double MAX_POSITION_DEGREES = 200.0;

  /**
   * The value to offset target positions by, in degrees (if zero, 0 will be at break beam tab).
   * This is used to account for the physical offset of where the break beam sensor and tab are (the
   * beam isn't broken when the turret is straight forward)
   */
  public static double TARGET_OFFSET_DEGREES = -20.0;

  /** How large of a time window to monitor encoder value for change over, in milliseconds */
  public static double ENCODER_DISCONNECT_WINDOW = 1000;

  /** The minimum number of (identical) encoder readings to base a disconnected state on */
  public static int ENCODER_DISCONNECT_MIN_READINGS = 25;

  /** The overcurrent threshold for the turret motor, in amps */
  public static double MAX_OPERATIONAL_MOTOR_CURRENT = 10.0;

  /** The maximum power to clamp the motor to if the encoder is thought to be disconnected */
  public static double ENCODER_DISCONNECT_MAX_POWER = 0.2;

  /** Turret PID tuning constants */
  public static double KP = 0.03, KI = 0.001, KD = 0.0015; // tuned well enough for now

  // **Final vars***********************************************************************************
  /** The motor driving the turret */
  private final DcMotorEx motor;

  /** The break beam used to zero the turret's position */
  private final BreakBeam breakBeam;

  /** A PID controller to control the turret motor */
  private final PIDController controller;

  /** A Follower to enable slow turret movement */
  private final Follower targetFollower;

  /** A Follower to approximately simulate the worst acceptable turret movement */
  private final Follower positionSimulation;

  /** Used to track if encoder values change at all over time (no change indicates disconnect) */
  private final TimeWindowAverage encoderTracker;

  /** Telemetry to send debug / tuning info to */
  private final TelemetryManager tlm;

  // **Static vars**********************************************************************************
  /** Cached value of current alert threshold, to avoid unnecessary hardware writes */
  private static double oldMaxOperationalCurrent = 10.0;

  // **Normal vars**********************************************************************************
  /** The current state of the turret */
  private State state = State.DISABLED;

  /** The current operational status of the turret */
  private SystemStatus status = SystemStatus.UNKNOWN;

  /** The state of homing the turret. Only applies while actually homing */
  private HomingState homingState = HomingState.FORWARD;

  /** The position the turret is currently moving to, in degrees (encoder space, not robot space) */
  private double targetPositionDegrees = 0.0;

  /** The position the PID controller of the turret is moving to, in degrees (encoder space) */
  private double subTargetPositionDegrees = 0.0;

  /** The position the turret is currently at, in degrees (encoder space, not robot space) */
  private double currentPositionDegrees = 0.0;

  // todo fix flip around

  public Turret(HardwareMap hwMap, String motorName, String breakBeamName) {
    this.motor = hwMap.get(DcMotorEx.class, motorName);
    this.breakBeam = new BreakBeam(hwMap, breakBeamName);
    this.controller = new PIDController(KP, KI, KD);
    this.targetFollower = new Follower(0.0, 0.0, TOLERANCE_DEGREES, NORMAL_DEGREES_PER_SECOND);
    this.positionSimulation =
        new Follower(0.0, 0.0, TOLERANCE_DEGREES, POSITION_SIM_DEGREES_PER_SECOND);
    this.encoderTracker = new TimeWindowAverage((long) ENCODER_DISCONNECT_WINDOW, 1);
    this.tlm = PanelsTelemetry.INSTANCE.getTelemetry();
  }

  @Override
  public void init() {
    motor.setMotorDisable();
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setDirection(DcMotor.Direction.FORWARD); // might need to reverse
    // home on init for Auto OpModes
    if (OpModeInfo.type == OpModeType.AUTO) {
      enable();
      home();
    }
  }

  @Override
  public void initLoop() {
    // this is slightly unorthodox, but is the simplest way to make homing on init work
    if (state != State.DISABLED) {
      loop();
    }
  }

  @Override
  public void start() {
    if (state == State.DISABLED) enable();
  }

  @Override
  public void loop() {
    updateCurrentPosition();

    if (state == State.DISABLED) return; // if disabled, just read encoder

    updateStateSpecificBehavior();
    updateTargetPositions();
    updateTuningValues();
    updateStatus();
    if (TELEMETRY_ENABLE) updateTelemetry();
    updateMotorPower();
  }

  @Override
  public void stop() {
    disable();
  }

  @Override
  public SystemReport getStatus() {
    return new SystemReport(status);
  }

  /**
   * Sets the target position of the turret, in degrees
   *
   * @param degrees the target position of the turret
   */
  public void setPosition(double degrees, boolean allowFlipping) {
    if (Double.isNaN(degrees) || Double.isInfinite(degrees)) {
      throw new IllegalArgumentException("Degrees must be a finite number");
    }

    if (state == State.HOMING) return; // disallow setting position if homing

    // todo test

    // the current position of the turret relative to the robot (0 = forward)
    double currentDegreesRobot = currentPositionDegrees - TARGET_OFFSET_DEGREES;

    degrees = normalizeAround(degrees, 0.0); // normalize to be in nice number space

    // we assume that min is not less than -360, and max is not more than +360, so best value is
    // not more than 360 degrees different from normalized input
    double[] possiblePositions = {degrees, degrees + 360.0, degrees - 360.0};
    double closestInBounds = Double.NaN; // for possible positions with flipping
    double closest = Double.NaN; // for impossible positions or no flipping
    double bestDistInBounds = Double.POSITIVE_INFINITY;
    double bestDist = Double.POSITIVE_INFINITY;

    // find closest position within bounds
    for (double i : possiblePositions) {
      double dist = Math.abs(i - currentDegreesRobot);
      if (dist < bestDist) { // could be out of bounds
        closest = i;
        bestDist = dist;
      }

      if (i >= MIN_POSITION_DEGREES && i <= MAX_POSITION_DEGREES) { // if in bounds
        if (dist < bestDistInBounds) { // is in bounds
          closestInBounds = i;
          bestDistInBounds = dist;
        }
      }
    }

    if (Double.isNaN(closestInBounds) || !allowFlipping) { // target out of bounds, or no flipping
      // if we can't flip around, or if the target is impossible, use closest and constrain
      degrees = MathUtils.clamp(closest, MIN_POSITION_DEGREES, MAX_POSITION_DEGREES);

    } else { // target position is reachable
      degrees = closestInBounds;
    }

    targetPositionDegrees = degrees + TARGET_OFFSET_DEGREES; // apply target offset
  }

  /** Starts homing the turret, recalibrating its "zero position" */
  public void home() {
    state = State.HOMING;
    resetEncoder(); // reset motor encoder
    homingState = HomingState.FORWARD; // start by moving +180 degrees from current position
    targetPositionDegrees = homingState.target;
    targetFollower.setTolerance(0.0);
    targetFollower.setSpeed(HOMING_DEGREES_PER_SECOND);
  }

  /**
   * Enables turret motion
   *
   * @apiNote This method is exposed in case it is useful, but is called automatically, so most of
   *     the time it isn't necessary to call it
   */
  public void enable() {
    motor.setMotorEnable();
    updateCurrentPosition();
    positionSimulation.setValue(currentPositionDegrees);
    positionSimulation.setTarget(currentPositionDegrees);
    targetFollower.setValue(currentPositionDegrees);
    targetFollower.setTarget(currentPositionDegrees);
    targetPositionDegrees = currentPositionDegrees;
    controller.reset();
    state = State.NORMAL;
  }

  /** Disables turret motion */
  public void disable() {
    motor.setMotorDisable();
    state = State.DISABLED;
  }

  /**
   * Gets if the turret is at its target
   *
   * @return true if the turret is at target, false if moving to its target, homing, or disabled
   */
  public boolean atTarget() {
    if (state == State.HOMING || state == State.DISABLED) return false;
    return atTargetInternal();
  }

  /**
   * Gets if the turret is at its target position, regardless of state
   *
   * @return true if at target, otherwise false
   */
  private boolean atTargetInternal() {
    return areEqual(currentPositionDegrees, targetPositionDegrees, TOLERANCE_DEGREES);
  }

  /** Updates configurable tuning values */
  private void updateTuningValues() {
    positionSimulation.setSpeed(POSITION_SIM_DEGREES_PER_SECOND);
    controller.setPID(KP, KI, KD);
    encoderTracker.setWindowSpan((long) ENCODER_DISCONNECT_WINDOW);

    if (!areEqual(MAX_OPERATIONAL_MOTOR_CURRENT, oldMaxOperationalCurrent)) {
      motor.setCurrentAlert(MAX_OPERATIONAL_MOTOR_CURRENT, CurrentUnit.AMPS);
      oldMaxOperationalCurrent = MAX_OPERATIONAL_MOTOR_CURRENT;
    }
  }

  /** Updates telemetry sent from the turret */
  private void updateTelemetry() {
    tlm.addData("Turret target pos", targetPositionDegrees);
    tlm.addData("Turret raw simulated pos", positionSimulation.getValue());
    tlm.addData("Turret raw current pos", currentPositionDegrees);
    tlm.addData("Turret pos ticks", motor.getCurrentPosition());
    tlm.addData("Turret PID target", subTargetPositionDegrees);
    tlm.addData("Turret state", state);
    tlm.addData("Turret status", status);
    tlm.addData("Turret encoder reading #", encoderTracker.getNumReadings());
    tlm.addData("Turret motor overcurrent", motor.isOverCurrent());
    tlm.addData("Turret sensor", breakBeam.getState());
  }

  /** Reads and updates the current position of the turret (not normalized) */
  private void updateCurrentPosition() {
    int currentPosTicks = motor.getCurrentPosition();
    currentPositionDegrees = ticksToAngleUnits(currentPosTicks, TICKS_PER_REV, AngleUnit.DEGREES);
    encoderTracker.addReading(currentPosTicks);
  }

  /** Resets the turret's motor's encoder to consider the current position to be 0 */
  private void resetEncoder() {
    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    currentPositionDegrees = 0.0;
    positionSimulation.setValue(currentPositionDegrees);
    positionSimulation.setTarget(currentPositionDegrees);
    targetFollower.setValue(currentPositionDegrees);
    targetFollower.setTarget(currentPositionDegrees);
    controller.reset();
  }

  /** Calculates and applies the correct motor power for the turret */
  private void updateMotorPower() {
    double targetMotorPower = controller.calculate(currentPositionDegrees);
    // if the encoder is disconnected, clamp motor power to avoid damage
    // we don't fully turn off the motor, in case the disconnect is a false positive (unlikely)
    if (status == SystemStatus.INOPERABLE) {
      targetMotorPower =
          clamp(targetMotorPower, -ENCODER_DISCONNECT_MAX_POWER, ENCODER_DISCONNECT_MAX_POWER);
    }

    // avoid setting motor powers that are basically the same as before, to help loop times
    if (!areEqual(targetMotorPower, motor.getPower(), MOTOR_UPDATE_TOLERANCE)) {
      motor.setPower(targetMotorPower);
    }
  }

  /**
   * Calculates the correct sub-target position (for slowed movement), and applies it to the PID
   * controller and the position simulation
   */
  private void updateTargetPositions() {
    targetFollower.setTarget(targetPositionDegrees);
    subTargetPositionDegrees = targetFollower.getValue();

    positionSimulation.setTarget(subTargetPositionDegrees);
    controller.setSetPoint(subTargetPositionDegrees);
  }

  /** Handles state-specific behavior, like homing */
  private void updateStateSpecificBehavior() {
    switch (state) {
      case DISABLED:
        break;

      case HOMING:
        if (breakBeam.isBroken()) { // tab at zero point is breaking sensor beam, thus, we are at 0
          resetEncoder();
          state = State.NORMAL;
          targetPositionDegrees = 0.0;
          // intentional fallthrough
        } else { // beam remains unbroken, thus, we still don't know where we are
          // if turret has moved fully one way, reverse
          if (targetFollower.isAtTarget() && atTargetInternal()) {
            homingState =
                homingState == HomingState.FORWARD ? HomingState.BACK : HomingState.FORWARD;
            targetPositionDegrees = homingState.target;
          }
          break;
        }

      case NORMAL: // not homing, so safe to set normal speed and tolerance values
        targetFollower.setSpeed(NORMAL_DEGREES_PER_SECOND);
        targetFollower.setTolerance(TOLERANCE_DEGREES);
        break;
    }
  }

  /** Updates the current operational status of the turret */
  private void updateStatus() {
    switch (state) {
      case DISABLED:
        status = SystemStatus.INACTIVE;
        break;

      case HOMING:
      case NORMAL:
        boolean atSubTarget =
            areEqual(currentPositionDegrees, subTargetPositionDegrees, TOLERANCE_DEGREES);
        boolean simAtSubTarget = positionSimulation.isAtTarget();

        if (areEqual(encoderTracker.getMaxMinReadingDifference(), 0.0)
            && encoderTracker.getNumReadings() >= ENCODER_DISCONNECT_MIN_READINGS) {
          status = SystemStatus.INOPERABLE; // encoder disconnected
        } else if ((simAtSubTarget && !atSubTarget) || motor.isOverCurrent()) {
          status = SystemStatus.COMPROMISED; // turret jammed or stuck
        } else {
          status = SystemStatus.NOMINAL;
        }
    }
  }
}
