/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import static org.firstinspires.ftc.teamcode.constants.UpdateTolerance.MOTOR_UPDATE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.utils.MathUtils.angleUnitsPerSecondToRpm;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.interfaces.MonitoredSystem;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.types.SystemStatus;
import org.firstinspires.ftc.teamcode.utils.TimeWindowAverage;

/**
 * Class to control an active intake
 *
 * @apiNote this class leaves some things, such as automatic ejecting, to the code using it
 * @implNote this assumes that bulk reading is enabled and telemetry is updated elsewhere
 */
@Configurable
public class Intake implements MonitoredSystem, System {
  public enum State {
    DISABLED(0, 0, 8),
    ACTIVE(1, 240, 8),
    EJECTING(-0.7, -120, 6);

    State(double motorPower, double targetMinRpm, double targetMaxCurrent) {
      this.motorPower = motorPower;
      this.targetMinRpm = targetMinRpm;
      this.targetMaxCurrent = targetMaxCurrent;
    }

    public final double motorPower;
    public final double targetMinRpm;
    public final double targetMaxCurrent;
  }

  /** If intake telemetry and tunable values should be used and updated each loop */
  public static boolean TUNING = false;
  /** The length of time to take averages over */
  public static long AVERAGE_WINDOW_SPAN = 500;
  /** The minimum number of readings to base an average off of */
  public static int AVERAGE_MIN_READINGS = 10;
  /** The amount of time (ms) after a state change that RPM readings are ignored */
  public static double STATE_CHANGE_AVERAGE_LOCKOUT_TIME = 250;
  /** The amount of time (ms) after a state change that status is assumed to be nominal */
  public static double STATE_CHANGE_STATUS_LOCKOUT_TIME = 750;

  /** The angle unit to be used for motor operations */
  private static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

  /** The intake's motor */
  private final DcMotorEx motor;
  /** The current state of the intake */
  private State state = State.DISABLED;
  /** The current operational status of the intake */
  private SystemStatus status = SystemStatus.INACTIVE;
  /** Timer to track how long the current state has been active */
  private final ElapsedTime stateTime;
  /** An average of the actual speed of the intake, to detect stalls */
  private final TimeWindowAverage rpmAverage;
  /** Telemetry to send data to */
  private final TelemetryManager tlm;
  /** If the intake is stalled */
  private boolean stalled = false;

  public Intake(HardwareMap hwMap, String motorName) throws IllegalArgumentException {
    if (hwMap == null || motorName == null) {
      throw new IllegalArgumentException("Hardware map and motor name must not be null");
    }
    motor = hwMap.get(DcMotorEx.class, motorName);
    stateTime = new ElapsedTime();
    rpmAverage = new TimeWindowAverage(AVERAGE_WINDOW_SPAN, AVERAGE_MIN_READINGS);
    tlm = PanelsTelemetry.INSTANCE.getTelemetry();
  }

  @Override
  public void init() {
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor.setDirection(DcMotor.Direction.FORWARD);
    motor.setCurrentAlert(state.targetMaxCurrent, CurrentUnit.AMPS);
    motor.setPower(state.motorPower);
    motor.setMotorDisable();
    stateTime.reset();
  }

  @Override
  public void initLoop() {

  }

  @Override
  public void start() {
    motor.setMotorEnable();
    updateConfigParameters();
    stateTime.reset();
  }

  @Override
  public void loop() {
    updateConfigParameters();

    updateMotorPower();

    if (stateTime.milliseconds() >= STATE_CHANGE_AVERAGE_LOCKOUT_TIME) {
      rpmAverage.addReading(angleUnitsPerSecondToRpm(ANGLE_UNIT, motor.getVelocity(ANGLE_UNIT)));
    }

    updateStatus();
    updateTelemetry();
  }

  @Override
  public void stop() {
    motor.setMotorDisable();
    state = State.DISABLED;
    status = SystemStatus.INACTIVE;
  }

  @Override
  public SystemReport getStatus() {
    return new SystemReport(status);
  }

  /**
   * Gets if the intake is stalled
   *
   * @return true if the intake is stalled, false if it isn't
   */
  public boolean isStalled() {
    return stalled;
  }

  /**
   * Sets the state of the intake
   *
   * @param state the intakes new state
   */
  public void setState(State state) throws IllegalArgumentException {
    if (state == null) {
      throw new IllegalArgumentException("State must not be null");
    }
    if (this.state == state) return;

    // start timing how long this state has been active
    stateTime.reset();

    // we are changing the target speed, so past readings will be "bad"
    rpmAverage.clearAverage();
    stalled = false;
    motor.setCurrentAlert(state.targetMaxCurrent, CurrentUnit.AMPS);
    this.state = state;

    updateMotorPower();
  }

  /**
   * Gets the state of the intake
   *
   * @return the intake's state
   */
  public State getState() {
    return state;
  }

  /**
   * Applies the correct power to the motor
   */
  private void updateMotorPower() {
    if (Math.abs(motor.getPower() - state.motorPower) > MOTOR_UPDATE_TOLERANCE) {
      motor.setPower(state.motorPower);
    }
  }

  /**
   * Updates the tuning parameters of the intake, so they update if tuned
   */
  private void updateConfigParameters() {
    if (!TUNING) return; // only update tunable values if tuning
    rpmAverage.setMinReadings(AVERAGE_MIN_READINGS);
    rpmAverage.setWindowSpan(AVERAGE_WINDOW_SPAN);
  }

  /**
   * Updates the intake's operational status
   */
  private void updateStatus() {
    TimeWindowAverage.Average avgRpm = rpmAverage.getAverage();
    boolean statusUnlocked = stateTime.milliseconds() > STATE_CHANGE_STATUS_LOCKOUT_TIME;
    double minRpm = state.targetMinRpm;
    boolean rpmBelowMin = minRpm >= 0 ? avgRpm.average < minRpm : avgRpm.average > minRpm;

    if (state == State.DISABLED) {
      status = SystemStatus.INACTIVE;
      stalled = false;

    } else if (statusUnlocked && !avgRpm.confident() && !motor.isOverCurrent()) {
      status = SystemStatus.UNKNOWN;
      stalled = false;

    } else if (statusUnlocked && (motor.isOverCurrent() || rpmBelowMin)) {
      status = SystemStatus.COMPROMISED;
      stalled = true;

    } else {
      status = SystemStatus.NOMINAL;
      stalled = false;
    }
  }

  /**
   * Updates telemetry data sent from the intake
   */
  private void updateTelemetry() {
    if (!TUNING) return; // only show telemetry if tuning
    tlm.addData("State", state);
    tlm.addData("Status", status);
    tlm.addData("Avg. RPM", rpmAverage.getAverage().average);
    tlm.addData("Avg. reading #", rpmAverage.getAverage().readingNumber);
    tlm.addData("Motor overcurrent", motor.isOverCurrent());
  }
}
