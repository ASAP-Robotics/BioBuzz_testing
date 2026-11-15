/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.types.SystemStatus;

/**
 * Class to control an active intake
 *
 * @apiNote this class leaves some things, such as automatic ejecting, to the code using it
 */
@Configurable
public class Intake extends SpinningThing {
  public enum State {
    DISABLED(0, 0, 8),
    INTAKING(1, 533, 8),
    EJECTING(-0.7, -373, 6);

    State(double motorPower, double targetMinRpm, double targetMaxCurrent) {
      this.motorPower = motorPower;
      this.targetMinRpm = targetMinRpm;
      this.targetMaxCurrent = targetMaxCurrent;
    }

    public final double motorPower;
    public final double targetMinRpm;
    public final double targetMaxCurrent;
  }

  /** If intake telemetry should be updated each loop */
  public static boolean TELEMETRY_ENABLE = false;

  /** The current state of the intake */
  protected State state = State.DISABLED;

  /** Timer to track how long the current state has been active */
  protected final ElapsedTime stateTime;

  /** If the intake is stalled */
  protected boolean stalled = false;

  public Intake(HardwareMap hwMap, String motorName) {
    super(hwMap, motorName);
    stateTime = new ElapsedTime();
    }

  @Override
  public void init() {
    currentAlertVal = state.targetMaxCurrent;
    targetMotorPower = state.motorPower;

    super.init();

    motor.setDirection(DcMotorSimple.Direction.REVERSE);

    stateTime.reset();
  }

  @Override
  public void start() {
    super.start();

    stateTime.reset();
  }

  @Override
  public void loop() {
    targetMotorPower = state.motorPower;
    currentAlertVal = state.targetMaxCurrent;
    speedAlertVal = state.targetMinRpm;

    super.loop();
  }

  @Override
  public void stop() {
    super.stop();
    state = State.DISABLED;
    status = SystemStatus.INACTIVE;
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

    stalled = false; // reset stalled status
    this.state = state;
    stateTime.reset();
  }

  /**
   * Gets the state of the intake
   *
   * @return the intake's state
   */
  public State getState() {
    return state;
  }


  /** Updates the intake's operational status */
  protected void updateStatus() {
    boolean statusUnlocked = stateTime.milliseconds() > AVERAGE_WINDOW_SPAN;

    if (state == State.DISABLED) {
      status = SystemStatus.INACTIVE;
      stalled = false;

    } else if (encoderDisconnected) {
      status = SystemStatus.FALLBACK; // disconnected encoder
      stalled = overcurrent;

    } else if ((statusUnlocked && underspeed) || overcurrent) {
      status = SystemStatus.COMPROMISED; // underspeed or overcurrent
      stalled = true;

    } else {
      status = SystemStatus.NOMINAL;
      stalled = false;
    }
  }

  /** Updates telemetry data sent from the intake */
  protected void updateTelemetry() {
    if (!TELEMETRY_ENABLE) return; // only show telemetry if tuning
    tlm.addData("Intake state", state);
    tlm.addData("Intake status", status);
    tlm.addData("Intake avg. RPM", rpmAverage.getAverage().average);
    tlm.addData("Intake avg. reading #", rpmAverage.getNumReadings());
    tlm.addData("Intake motor overcurrent", overcurrent);
    tlm.addData("Intake stalled", stalled);
  }
}
