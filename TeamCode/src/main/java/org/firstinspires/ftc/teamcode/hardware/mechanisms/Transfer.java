package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.sensors.BreakBeam;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;
import org.firstinspires.ftc.teamcode.types.SystemStatus;

/**
 * Class to control the transfer between the front intake and the turret.
 */
public class Transfer extends SpinningThing {
  public enum State {
    STANDBY(0, 0, 8, true),
    LIFTING(1, 383, 8, true),
    EJECTING(-0.7, -268, 6, true),
    SHOOTING(1, 383, 8, false);

    State(double motorPower, double targetMinRpm, double targetMaxCurrent, boolean blockerDeployed) {
      this.motorPower = motorPower;
      this.targetMinRpm = targetMinRpm;
      this.targetMaxCurrent = targetMaxCurrent;
      this.blockerDeployed = blockerDeployed;
    }

    public final double motorPower;
    public final double targetMinRpm;
    public final double targetMaxCurrent;
    public final boolean blockerDeployed;
  }

  /** If transfer telemetry should be updated each loop */
  public static boolean TELEMETRY_ENABLE = false;

  /** The amount of time to lift pollen for when intaking */
  public static double LIFT_TIME_MS = 1500;

  /** The amount of time to eject for when (manually) ejecting */
  public static double EJECT_TIME_MS = 750;

  /** The amount of time to lift pollen for when shooting */
  public static double SHOOT_TIME_MS = 2500;

  /** The servo position when the blocker is retracted (degrees) */
  public static double BLOCKER_RETRACTED_POSITION = 0.0; // todo tune

  /** The servo position when the blocker is deployed (degrees) */
  public static double BLOCKER_DEPLOYED_POSITION = 60.0; // todo tune

  /** The sensor in front of th transfer to detect pollen */
  private final BreakBeam sensor;

  /** The Axon moving the top blocker that prevents pollen from entering the turret */
  private final Axon topBlocker;

  /** Timer to track how long the current state has been active */
  protected final ElapsedTime stateTime;

  /** The time since the state was last set (even if it was re-set tot the same state) */
  private final ElapsedTime stateSetTime;

  /** The operating state of the transfer */
  private State state = State.STANDBY;

  /** Last read state of the sensor, for change detection */
  private BreakBeam.State oldSensorState = BreakBeam.State.UNBROKEN;

  public Transfer(HardwareMap hwMap,
                  String motorName,
                  String sensorName,
                  String servoName,
                  String servoEncoderName) {
    super(hwMap, motorName);
    this.sensor = new BreakBeam(hwMap, sensorName);
    this.topBlocker = new Axon(Axon.Type.MAX, hwMap, servoName, servoEncoderName);
    this.stateTime = new ElapsedTime();
    this.stateSetTime = new ElapsedTime(Long.MAX_VALUE / 2); // don't turn on upon start
  }

  @Override
  public void loop() {
    BreakBeam.State sensorState = sensor.getState();
    if (state != State.SHOOTING &&
        sensorState == BreakBeam.State.BROKEN &&
        oldSensorState == BreakBeam.State.UNBROKEN) {
      setState(State.LIFTING); // start lifting pollen further up into robot if present
    }
    oldSensorState = sensorState;

    if (state == State.EJECTING && stateTime.milliseconds() > EJECT_TIME_MS) {
      setState(State.STANDBY);
    } else if (state == State.SHOOTING && stateTime.milliseconds() > SHOOT_TIME_MS) {
      setState(State.STANDBY);
    } else if (state == State.LIFTING && stateSetTime.milliseconds() > LIFT_TIME_MS) {
      setState(State.STANDBY);
    }

    targetMotorPower = topBlocker.atTarget() ? state.motorPower : 0.0; // don't jam blocker

    super.loop();
  }

  /**
   * Starts lifting pollen into turret to shoot
   */
  public void shoot() {
    setState(State.SHOOTING);
  }

  /**
   * Sets the operating state of the transfer
   *
   * @param state the transfer's state
   */
  private void setState(State state) {
    if (state == null) {
      throw new IllegalArgumentException("State must not be null");
    }
    if (this.state != state) stateTime.reset();

    this.state = state;

    targetMotorPower = state.motorPower; // might be redundant, but harmless
    currentAlertVal = state.targetMaxCurrent;
    speedAlertVal = state.targetMinRpm;

    topBlocker.setPosition(
        state.blockerDeployed ? BLOCKER_DEPLOYED_POSITION : BLOCKER_RETRACTED_POSITION
    );

    stateSetTime.reset();
  }

  @Override
  protected void updateStatus() {
    if (state == State.STANDBY) {
      status = SystemStatus.INACTIVE;

    } else if (overcurrent) {
      status = SystemStatus.FALLBACK;

    } else {
      status = SystemStatus.NOMINAL;
    }
  }

  /** Updates telemetry data sent from the transfer */
  @Override
  protected void updateTelemetry() {
    if (!TELEMETRY_ENABLE) return; // only show telemetry if tuning
    tlm.addData("Transfer state", state);
    tlm.addData("Transfer status", status);
    tlm.addData("Transfer avg. RPM", rpmAverage.getAverage().average);
    tlm.addData("Transfer avg. reading #", rpmAverage.getNumReadings());
    tlm.addData("Transfer motor overcurrent", overcurrent);
    tlm.addData("Transfer sensor state", oldSensorState);
  }
}
