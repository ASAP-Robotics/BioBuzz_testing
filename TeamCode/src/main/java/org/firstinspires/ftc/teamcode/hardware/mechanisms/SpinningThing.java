package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import static org.firstinspires.ftc.teamcode.constants.Tolerances.MOTOR_CURRENT_TOLERANCE_A;
import static org.firstinspires.ftc.teamcode.constants.Tolerances.MOTOR_UPDATE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.utils.MathUtils.angleUnitsPerSecondToRpm;

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
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.TimeWindowAverage;

/**
 * Class to lay out the basics of a system that consists of a thing that spins, like an intake
 *
 * @implNote this assumes that bulk reading is enabled and telemetry is updated elsewhere
 */
@Configurable
public abstract class SpinningThing implements System, MonitoredSystem {
  /** The length of time to take averages over (also used for encoder disconnect detection) */
  public static long AVERAGE_WINDOW_SPAN = 1000;

  /** The minimum number of readings to base an average or encoder disconnect off of */
  public static int AVERAGE_MIN_READINGS = 25;

  /** The maximum power to apply to the motor if it is overcurrent */
  public static double OVERCURRENT_MAX_MOTOR_POWER = 0.75;



  /** The angle unit to be used for motor operations */
  protected static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

  /** The motor driving the spinning thing */
  private final DcMotorEx motor;

  /** An average of the speed of the spinning thing, to detect stalls and encoder disconnects*/
  protected final TimeWindowAverage rpmAverage;

  /** Telemetry to send data to */
  protected final TelemetryManager tlm;

  /** The current operational status of the system */
  protected SystemStatus status = SystemStatus.INACTIVE;

  /** If the spinning thing is stalled (abnormally low speed) */
  protected boolean underspeed = false;

  /** If the motor's current is too high */
  protected boolean overcurrent = false;

  /** If the motor's encoder is detected as being disconnected */
  protected boolean encoderDisconnected = false;

  /** The current level that will trigger an overcurrent alert, in amps */
  protected double currentAlertVal = 10.0;

  /** The minimum speed of the motor to not trigger an underspeed alert */
  protected double speedAlertVal = 0.0;

  /** The target power of the motor, ignoring things like caps for overcurrent alarms */
  protected double targetMotorPower = 0.0;

  /** The last set motor current alert, for loop time optimization */
  private double oldMotorCurrentAlert = Double.NaN;

  public SpinningThing(HardwareMap hwMap, String motorName) {
    if (hwMap == null || motorName == null) {
      throw new IllegalArgumentException("Hardware map and motor name must not be null");
    }

    this.motor = hwMap.get(DcMotorEx.class, motorName);
    rpmAverage = new TimeWindowAverage(AVERAGE_WINDOW_SPAN, AVERAGE_MIN_READINGS);
    tlm = PanelsTelemetry.INSTANCE.getTelemetry();
  }

  @Override
  public void init() {
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor.setDirection(DcMotor.Direction.FORWARD);
    motor.setCurrentAlert(currentAlertVal, CurrentUnit.AMPS);
    motor.setPower(targetMotorPower);
    motor.setMotorDisable();
  }

  @Override
  public void initLoop() {

  }

  @Override
  public void start() {
    motor.setMotorEnable();
  }

  @Override
  public void loop() {
    updateTuningVals();
    updateMotorCurrentAlerts();
    updateMotorPower();
    updateMotorVelocity();
    updateMotorStatus();
    updateStatus();
    updateTelemetry();
  }

  @Override
  public void stop() {
    motor.setMotorDisable();
  }

  @Override
  public SystemReport getStatus() {
    return new SystemReport(status);
  }

  /**
   * Updates tuning values so changes are applied
   */
  protected void updateTuningVals() {
    rpmAverage.setMinReadings(AVERAGE_MIN_READINGS);
    rpmAverage.setWindowSpan(AVERAGE_WINDOW_SPAN);
  }

  /**
   * Calculates the correct power to set the motor to, and sets it
   */
  private void updateMotorPower() {
    double maxMotorPower = overcurrent ? OVERCURRENT_MAX_MOTOR_POWER : Double.POSITIVE_INFINITY;
    double motorPower = MathUtils.clamp(targetMotorPower, -maxMotorPower, maxMotorPower);
    if (!MathUtils.areEqual(motor.getPower(), motorPower, MOTOR_UPDATE_TOLERANCE)) {
      motor.setPower(motorPower);
    }
  }

  /**
   * Updates the current operational status of the system
   */
  protected abstract void updateStatus();

  /**
   * Sends telemetry
   */
  protected abstract void updateTelemetry();

  /**
   * Reads and handles the motor's velocity
   */
  private void updateMotorVelocity() {
    rpmAverage.addReading(angleUnitsPerSecondToRpm(ANGLE_UNIT, motor.getVelocity(ANGLE_UNIT)));
  }

  /**
   * Updates the motor's overcurrent and underspeed status, and the encoder's connection status
   */
  private void updateMotorStatus() {
    double avgRpm = rpmAverage.getAverage().average;
    underspeed = speedAlertVal >= 0 ? avgRpm < speedAlertVal : avgRpm > speedAlertVal;
    overcurrent = motor.isOverCurrent();
    encoderDisconnected =
        rpmAverage.getMaxMinReadingDifference() == 0.0 &&
            rpmAverage.getNumReadings() >= AVERAGE_MIN_READINGS;
  }

  /**
   * Updates the motor's current alert level
   */
  private void updateMotorCurrentAlerts() {
    if (!MathUtils.areEqual(oldMotorCurrentAlert, currentAlertVal, MOTOR_CURRENT_TOLERANCE_A)) {
      motor.setCurrentAlert(currentAlertVal, CurrentUnit.AMPS);
      oldMotorCurrentAlert = currentAlertVal;
    }
  }
}
