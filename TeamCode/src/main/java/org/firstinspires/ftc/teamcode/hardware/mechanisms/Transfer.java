/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.sensors.BreakBeam;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;
import org.firstinspires.ftc.teamcode.types.SystemStatus;

/** Class to control the transfer between the front intake and the turret. */
@Configurable
public class Transfer extends SpinningThing {
  public enum State {
    DISABLED(0.0, 0.0, 6, false),
    INTAKING(1, 206, 6, true),
    EJECTING(-0.7, -144, 4, true),
    SHOOTING(1, 206, 6, false);

    State(
        double motorPower, double targetMinRpm, double targetMaxCurrent, boolean blockerDeployed) {
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
  public static double LIFT_TIME_MS = 1000;

  /** The servo position when the blocker is retracted (degrees) */
  public static double BLOCKER_RETRACTED_POSITION = 47.0;

  /** The servo position when the blocker is deployed (degrees) */
  public static double BLOCKER_DEPLOYED_POSITION = 0.0;

  /** The sensor in front of th transfer to detect pollen */
  private final BreakBeam sensor;

  /** The Axon moving the top blocker that prevents pollen from entering the turret */
  private final Axon topBlocker;

  /** The time since the sensor beam was last broken */
  private final ElapsedTime beamBrokenTimer;

  /** The operating state of the transfer */
  private State state = State.INTAKING;

  public Transfer(
      HardwareMap hwMap,
      String motorName,
      String sensorName,
      String servoName,
      String servoEncoderName) {
    super(hwMap, motorName);
    this.sensor = new BreakBeam(hwMap, sensorName);
    this.topBlocker = new Axon(Axon.Type.MAX, hwMap, servoName, servoEncoderName);
    this.beamBrokenTimer = new ElapsedTime();
  }

  @Override
  public void loop() {
    if (sensor.getState() == BreakBeam.State.BROKEN) {
      beamBrokenTimer.reset();
    }

    topBlocker.setPosition(
        state.blockerDeployed ? BLOCKER_DEPLOYED_POSITION : BLOCKER_RETRACTED_POSITION);

    boolean disable =
        !topBlocker.atTarget()
            || (state == State.INTAKING && beamBrokenTimer.milliseconds() > LIFT_TIME_MS);
    targetMotorPower = disable ? 0.0 : state.motorPower; // don't jam blocker
    currentAlertVal = state.targetMaxCurrent;
    speedAlertVal = disable ? 0.0 : state.targetMinRpm; // don't give false alerts

    super.loop();
  }

  /**
   * Sets the operating state of the transfer
   *
   * @param state the transfer's state
   */
  public void setState(State state) {
    if (state == null) {
      throw new IllegalArgumentException("State must not be null");
    }

    this.state = state;
  }

  /**
   * Gets the state of the transfer
   *
   * @return the transfer's state
   */
  public State getState() {
    return state;
  }

  @Override
  protected void updateStatus() {
    if (state == State.DISABLED) {
      status = SystemStatus.INACTIVE;

    } else if (overcurrent || underspeed) {
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
    tlm.addData("Transfer sensor time since broken", beamBrokenTimer.milliseconds());
  }
}
