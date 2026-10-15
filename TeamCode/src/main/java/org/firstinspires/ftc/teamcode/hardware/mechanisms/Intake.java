package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.constants.UpdateTolerance;
import org.firstinspires.ftc.teamcode.interfaces.MonitoredSystem;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.types.SystemStatus;
import org.firstinspires.ftc.teamcode.utils.TimeWindowAverage;

/**
 * Class to control an active intake
 *
 * @apiNote this class leaves some things, such as automatic ejecting, to the code using it
 */
@Configurable
public class Intake implements MonitoredSystem, System {
  public enum State {
    DISABLED(0, 0, 0),
    ACTIVE(1, 600, 5),
    EJECTING(-0.7, -420, 3.5);

    State(double motorPower, double targetMinSpeed, double targetMaxCurrent) {
      this.motorPower = motorPower;
      this.targetMinSpeed = targetMinSpeed;
      this.targetMaxCurrent = targetMaxCurrent;
    }

    public final double motorPower;
    public final double targetMinSpeed;
    public final double targetMaxCurrent;
  }

  /** The length of time to take averages over */
  public static long AVERAGE_WINDOW_SPAN = 500;
  /** The minimum number of readings to base an average off of */
  public static int AVERAGE_MIN_READINGS = 20;
  /** The amount of time to ignore readings for after a state change */
  public static double STATE_CHANGE_LOCKOUT_TIME = 500;

  /** The intake's motor */
  private final DcMotorEx motor;
  /** The current state of the intake */
  private State state = State.DISABLED;
  /** The current operational status of the intake */
  private SystemStatus status = SystemStatus.INACTIVE;
  /** Timer to track how long the current state has been active */
  private final ElapsedTime stateTime;
  /** An average of the actual speed of the intake, to detect stalls */
  private final TimeWindowAverage speedAverage;
  /** An average of the actual current draw of the intake, to detect stalls */
  private final TimeWindowAverage currentAverage;
  /** If the intake is stalled */
  private boolean stalled = false;

  public Intake(HardwareMap hwMap, String motorName) {
    motor = hwMap.get(DcMotorEx.class, motorName);
    stateTime = new ElapsedTime();
    speedAverage = new TimeWindowAverage(AVERAGE_WINDOW_SPAN, AVERAGE_MIN_READINGS);
    currentAverage = new TimeWindowAverage(AVERAGE_WINDOW_SPAN, AVERAGE_MIN_READINGS);
  }

  @Override
  public void init() {
    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    motor.setDirection(DcMotor.Direction.FORWARD);
    motor.setPower(state.motorPower);
    stateTime.reset();
  }

  @Override
  public void initLoop() {

  }

  @Override
  public void start() {
    updateAverageParameters();
    stateTime.reset();
  }

  @Override
  public void loop() {
    updateAverageParameters();

    if (Math.abs(motor.getPower() - state.motorPower) > UpdateTolerance.MOTOR_UPDATE_TOLERANCE) {
      motor.setPower(state.motorPower);
    }

    if (stateTime.milliseconds() >= STATE_CHANGE_LOCKOUT_TIME) {
      updateAverages();
    }

    updateStatus();
  }

  @Override
  public void stop() {
    motor.setPower(State.DISABLED.motorPower);
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
  public void setState(State state) {
    if (this.state == state) return;

    // start timing how long this state has been active
    stateTime.reset();

    // we are changing the target speed, so past readings will be "bad"
    speedAverage.clearAverage();
    currentAverage.clearAverage();
    stalled = false;

    this.state = state;
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
   * Updates the parameters of the motor monitoring averages, so they update if tuned
   */
  private void updateAverageParameters() {
    speedAverage.setMinReadings(AVERAGE_MIN_READINGS);
    speedAverage.setWindowSpan(AVERAGE_WINDOW_SPAN);
    currentAverage.setMinReadings(AVERAGE_MIN_READINGS);
    currentAverage.setWindowSpan(AVERAGE_WINDOW_SPAN);
  }

  /**
   * Updates the motor speed and current averages used for status monitoring
   *
   * @implNote this assumes that bulk reading is enabled elsewhere, otherwise it would be quite slow
   */
  private void updateAverages() {
    speedAverage.addReading(motor.getVelocity(AngleUnit.DEGREES));
    currentAverage.addReading(motor.getCurrent(CurrentUnit.AMPS));
  }

  /**
   * Updates the intake's operational status
   */
  private void updateStatus() {
    TimeWindowAverage.Average speedAverageValue = speedAverage.getAverage();
    TimeWindowAverage.Average currentAverageValue = currentAverage.getAverage();

    if (state == State.DISABLED) {
      status = SystemStatus.INACTIVE;
      stalled = false;

    } else if (!speedAverageValue.valid() || !currentAverageValue.valid()) {
      status = SystemStatus.UNKNOWN;
      // we might need to set if the intake is stalled here, but I'm leaving that out for now

    } else if (speedAverageValue.confident() &&
        currentAverageValue.confident() &&
        (state.targetMinSpeed < 0 ?
            speedAverageValue.average <= state.targetMinSpeed :
            speedAverageValue.average >= state.targetMinSpeed) &&
        Math.abs(currentAverageValue.average) <= state.targetMaxCurrent) {
      status = SystemStatus.NOMINAL;
      stalled = false;

    } else {
      status = SystemStatus.COMPROMISED;
      stalled = true;
    }
  }
}
