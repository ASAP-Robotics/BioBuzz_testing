/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import static org.firstinspires.ftc.teamcode.utils.MathUtils.angleUnitsPerSecondToRpm;
import static org.firstinspires.ftc.teamcode.utils.MathUtils.areEqual;
import static org.firstinspires.ftc.teamcode.utils.MathUtils.rpmToAngleUnitsPerSecond;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.interfaces.MonitoredSystem;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.types.SystemStatus;
import org.firstinspires.ftc.teamcode.utils.Follower;
import org.firstinspires.ftc.teamcode.utils.TimeWindowAverage;

/**
 * Class to control a flywheel
 *
 * @implNote this assumes that bulk reading is enabled and telemetry is updated elsewhere
 */
@Configurable
public class Flywheel implements System, MonitoredSystem {
  /** If flywheel telemetry and tunable values should be used and updated each loop */
  public static boolean TUNING = false;

  /** The speed that the RPM simulation moves at, in RPM per second */
  public static double RPM_SIM_UNITS_PER_SECOND = 100.0;

  /** The most error the actual flywheel RPM can have from its simulation and be "operational" */
  public static double MAX_OPERATIONAL_RPM_ERROR = 100.0;

  /** The most error the actual flywheel RPM can have from its target and be "at target" */
  public static double MAX_AT_TARGET_RPM_ERROR = 10.0;

  /** The overcurrent threshold for the flywheel motor, in amps */
  public static double MAX_OPERATIONAL_MOTOR_CURRENT = 10.0;

  /** Flywheel PIDF tuning constants */
  public static double KP = 0.0, KI = 0.0, KD = 0.0, KF = 0.0;

  /** The length of time to take averages over */
  public static long AVERAGE_WINDOW_SPAN = 500;

  /** The minimum number of readings to base an average off of */
  public static int AVERAGE_MIN_READINGS = 10;

  /** The angle unit to be used for motor operations */
  private static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

  /** The motor driving the flywheel */
  private final DcMotorEx motor;

  /** A simulation to approximate how the flywheel should normally behave */
  private final Follower rpmSimulation;

  /** An average of the past error of the flywheel RPM for use in status monitoring */
  private final TimeWindowAverage averageRpmError;

  /** The operational status of the flywheel */
  private SystemStatus status = SystemStatus.INACTIVE;

  /** Telemetry to send data to */
  private final TelemetryManager tlm;

  /** Cached values of PIDF tuning constants so they can be applied only when changed */
  private double oldKp = 0.0, oldKi = 0.0, oldKd = 0.0, oldKf = 0.0;

  /** The RPM that the flywheel should be spinning at */
  private double targetRpm = 0.0;

  /** The RPM that the flywheel is actually spinning at */
  private double actualRpm = 0.0;

  public Flywheel(HardwareMap hwMap, String motorName) throws IllegalArgumentException {
    if (hwMap == null || motorName == null) {
      throw new IllegalArgumentException("Hardware map and motor name must not be null");
    }
    this.motor = hwMap.get(DcMotorEx.class, motorName);
    this.rpmSimulation = new Follower(0.0, 0.0, 0.0, RPM_SIM_UNITS_PER_SECOND);
    this.averageRpmError = new TimeWindowAverage(AVERAGE_WINDOW_SPAN, AVERAGE_MIN_READINGS);
    this.tlm = PanelsTelemetry.INSTANCE.getTelemetry();
  }

  @Override
  public void init() {
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    motor.setDirection(DcMotor.Direction.FORWARD);
    motor.setVelocityPIDFCoefficients(KP, KI, KD, KF);
    motor.setVelocity(0, ANGLE_UNIT);
    motor.setCurrentAlert(MAX_OPERATIONAL_MOTOR_CURRENT, CurrentUnit.AMPS);
    motor.setMotorDisable();
  }

  @Override
  public void initLoop() {}

  @Override
  public void start() {
    motor.setMotorEnable();
    updateConfigParameters();
  }

  @Override
  public void loop() {
    motor.setVelocity(rpmToAngleUnitsPerSecond(ANGLE_UNIT, targetRpm), ANGLE_UNIT);
    rpmSimulation.setTarget(targetRpm);
    actualRpm = angleUnitsPerSecondToRpm(ANGLE_UNIT, motor.getVelocity(ANGLE_UNIT));
    averageRpmError.addReading(Math.abs(targetRpm - actualRpm));

    updateConfigParameters();
    updateStatus();
    updateTelemetry();
  }

  @Override
  public void stop() {
    motor.setMotorDisable();
    status = SystemStatus.INACTIVE;
  }

  @Override
  public SystemReport getStatus() {
    return new SystemReport(status);
  }

  /**
   * Sets the target RPM of the flywheel
   *
   * @param targetRpm the target RPM of the flywheel
   */
  public void setTargetRpm(double targetRpm) throws IllegalArgumentException {
    if (Double.isInfinite(targetRpm) || Double.isNaN(targetRpm)) {
      throw new IllegalArgumentException("Target RPM must be finite and not NaN");
    }
    if (areEqual(this.targetRpm, targetRpm)) return;

    this.targetRpm = targetRpm;
    rpmSimulation.setValue(actualRpm);
  }

  /**
   * Gets the target RPM of the flywheel
   *
   * @return the target RPM of the flywheel
   */
  public double getTargetRpm() {
    return targetRpm;
  }

  /**
   * Gets the actual RPM of the flywheel
   *
   * @return the actual RPM of the flywheel
   */
  public double getActualRpm() {
    return actualRpm;
  }

  /**
   * Gets if the flywheel is close enough to its target RPM to be considered "at target"
   *
   * @return if the flywheel is at its target RPM
   */
  public boolean isAtTargetRpm() {
    return areEqual(targetRpm, actualRpm, MAX_AT_TARGET_RPM_ERROR);
  }

  /** Updates the tuning parameters of the flywheel so they update if tuned */
  private void updateConfigParameters() {
    if (!TUNING) return; // only update tunable values if tuning
    rpmSimulation.setSpeed(RPM_SIM_UNITS_PER_SECOND);
    motor.setCurrentAlert(MAX_OPERATIONAL_MOTOR_CURRENT, CurrentUnit.AMPS);
    if (!areEqual(KP, oldKp)
        || !areEqual(KI, oldKi)
        || !areEqual(KD, oldKd)
        || !areEqual(KF, oldKf)) {
      motor.setVelocityPIDFCoefficients(KP, KI, KD, KF);
      oldKp = KP;
      oldKi = KI;
      oldKd = KD;
      oldKf = KF;
    }
    averageRpmError.setWindowSpan(AVERAGE_WINDOW_SPAN);
    averageRpmError.setMinReadings(AVERAGE_MIN_READINGS);
  }

  /** Updates the flywheel's operational status */
  private void updateStatus() {
    double simulatedMaxError =
        Math.abs(targetRpm - rpmSimulation.getValue()) + MAX_OPERATIONAL_RPM_ERROR;
    TimeWindowAverage.Average actualAverageRpmError = averageRpmError.getAverage();

    if (areEqual(targetRpm, 0)) {
      status = SystemStatus.INACTIVE;
    } else if (!actualAverageRpmError.valid()) {
      status = SystemStatus.UNKNOWN;
    } else if (actualAverageRpmError.average > simulatedMaxError || motor.isOverCurrent()) {
      status = SystemStatus.COMPROMISED;
    } else {
      status = SystemStatus.NOMINAL;
    }
  }

  /** Updates telemetry data sent from the flywheel */
  private void updateTelemetry() {
    if (!TUNING) return; // only show telemetry if tuning
    tlm.addData("Target RPM", targetRpm);
    tlm.addData("Actual RPM", actualRpm);
    tlm.addData("Simulated RPM", rpmSimulation.getValue());
    tlm.addData("Status", status);
    tlm.addData("Inst. RPM error", Math.abs(targetRpm - actualRpm));
    tlm.addData("Avg. RPM error", averageRpmError.getAverage().average);
    tlm.addData("Avg. reading #", averageRpmError.getAverage().readingNumber);
    tlm.addData("Motor overcurrent", motor.isOverCurrent());
  }
}
