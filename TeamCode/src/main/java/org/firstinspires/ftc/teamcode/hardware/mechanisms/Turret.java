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
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.types.SystemStatus;
import org.firstinspires.ftc.teamcode.utils.Follower;
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

  // **Constant
  // vals*********************************************************************************
  /** How many encoder ticks are in one full rotation of the turret (not the motor) */
  private static final double TICKS_PER_REV = 100.0; // todo use actual value

  // **Tunable
  // vals**********************************************************************************
  /** If telemetry from turret should be updated each loop */
  public static boolean TELEMETRY_ENABLE = false;

  /** The speed below which, if the turret moves at it, the turret will be considered stuck */
  public static double POSITION_SIM_DEGREES_PER_SECOND = 100.0; // todo tune

  /** The speed that the turret will attempt to move at when going to a set target */
  public static double NORMAL_DEGREES_PER_SECOND = Double.POSITIVE_INFINITY;

  /** The speed the turret will attempt to move at when homing */
  public static double HOMING_DEGREES_PER_SECOND = 60.0; // todo tune

  /** Roughly how close the turret can be to its target position and be "at target" */
  public static double TOLERANCE_DEGREES = 1.0;

  /** The center position of the turrets physical range of motion */
  public static double FLIP_POINT_DEGREES = 90.0;

  /** The value to offset target positions by, in degrees (if zero, 0 will be at break beam tab) */
  public static double TARGET_OFFSET_DEGREES = 0.0;

  /** How large of a time window to monitor encoder value for change over, in milliseconds */
  public static double ENCODER_DISCONNECT_WINDOW = 1000;

  /** The minimum number of (identical) encoder readings to base a disconnected state on */
  public static int ENCODER_DISCONNECT_MIN_READINGS = 25;

  /** The overcurrent threshold for the turret motor, in amps */
  public static double MAX_OPERATIONAL_MOTOR_CURRENT = 10.0;

  /** The maximum power to clamp the motor to if the encoder is thought to be disconnected */
  public static double ENCODER_DISCONNECT_MAX_POWER = 0.2;

  /** Turret PID tuning constants */
  public static double KP = 0.0, KI = 0.0, KD = 0.0; // todo tune

  // **Final
  // vars************************************************************************************
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

  // **Static
  // vars***********************************************************************************
  /** Cached value of current alert threshold, to avoid unnecessary hardware writes */
  private static double oldMaxOperationalCurrent = 10.0;

  // **Normal
  // vars***********************************************************************************
  /** The current state of the turret */
  private State state = State.DISABLED;

  /** The current operational status of the turret */
  private SystemStatus status = SystemStatus.UNKNOWN;

  /** The position the turret is currently moving to, in degrees */
  private double targetPositionDegrees = 0.0;

  /** The position the PID controller of the turret is moving to, in degrees */
  private double subTargetPositionDegrees = 0.0;

  /** The position the turret is currently at, in degrees */
  private double currentPositionDegrees = 0.0;

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
    motor.setDirection(DcMotor.Direction.FORWARD);
  }

  @Override
  public void initLoop() {
    // todo add ability to home on init
  }

  @Override
  public void start() {
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
    motor.setMotorDisable();
    state = State.DISABLED;
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
  public void setPosition(double degrees) {
    if (Double.isNaN(degrees) || Double.isInfinite(degrees)) {
      throw new IllegalArgumentException("Degrees must be a finite number");
    }
    if (state == State.HOMING) return; // disallow setting position if homing

    degrees += TARGET_OFFSET_DEGREES; // apply target offset
    targetPositionDegrees = normalizeAround(degrees, FLIP_POINT_DEGREES); // constrain value
  }

  /** Starts homing the turret, recalibrating its "zero position" */
  public void home() {
    state = State.HOMING;
    resetEncoder(); // reset motor encoder
    targetPositionDegrees = 180.0; // start moving 180 degrees from current position
    targetFollower.setTolerance(0.0);
    targetFollower.setSpeed(HOMING_DEGREES_PER_SECOND);
  }

  /**
   * Gets if the turret is at its target
   *
   * @return true if the turret is at target, false if moving to its target, homing, or disabled
   */
  public boolean atTarget() {
    if (state == State.HOMING || state == State.DISABLED) return false;
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
    tlm.addData("Turret simulated pos", positionSimulation.getValue());
    tlm.addData("Turret current pos", currentPositionDegrees);
    tlm.addData("Turret PID target", subTargetPositionDegrees);
    tlm.addData("Turret state", state);
    tlm.addData("Turret status", status);
    tlm.addData("Turret encoder reading #", encoderTracker.getNumReadings());
    tlm.addData("Turret motor overcurrent", motor.isOverCurrent());
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
          // if turret has moved 180 degrees one way, reverse
          if (targetFollower.isAtTarget() && atTarget()) {
            targetPositionDegrees = targetPositionDegrees == 180.0 ? -180.0 : 180.0;
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
