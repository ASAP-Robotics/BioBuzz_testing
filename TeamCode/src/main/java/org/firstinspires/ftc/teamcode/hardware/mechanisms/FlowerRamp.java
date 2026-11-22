/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;
import org.firstinspires.ftc.teamcode.interfaces.MonitoredSystem;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.types.SystemReport;

@Configurable
public class FlowerRamp implements MonitoredSystem, System {
  public enum State {
    /** Keeps the intake ramp within the starting configuration limits */
    STARTING,
    /** Keeps the intake ramp up and out of the way of shooting, etc. */
    STOWED,
    /** Deploys the intake ramp so the robot can intake from the flowers */
    DEPLOYED
  }

  public static boolean TELEMETRY_ENABLE = false;
  public static double STARTING_POS = 200.0;
  public static double STOWED_POS = 120.0;
  public static double DEPLOYED_POS = 5.0;
  public static double RIGHT_OFFSET = -5.0;
  public static double LEFT_OFFSET = 15.0;

  private final Axon right, left;
  private final TelemetryManager tlm;

  private State state = State.STARTING;

  public FlowerRamp(
      HardwareMap hwMap,
      String rightServoName,
      String rightEncoderName,
      String leftServoName,
      String leftEncoderName) {
    this.right = new Axon(Axon.Type.MAX, hwMap, rightServoName, rightEncoderName);
    this.left = new Axon(Axon.Type.MAX, hwMap, leftServoName, leftEncoderName);
    this.tlm = PanelsTelemetry.INSTANCE.getTelemetry();
  }

  @Override
  public void init() {
    updateStateBehavior();
    right.init();
    left.init();
    updateTelemetry();
  }

  @Override
  public void initLoop() {
    right.initLoop();
    left.initLoop();
    updateTelemetry();
  }

  @Override
  public void start() {
    updateStateBehavior();
    right.start();
    left.start();
    updateTelemetry();
  }

  @Override
  public void loop() {
    updateStateBehavior();
    right.loop();
    left.loop();
    updateTelemetry();
  }

  @Override
  public void stop() {
    right.stop();
    left.stop();
    updateTelemetry();
  }

  @Override
  public SystemReport getStatus() {
    SystemReport rightReport = right.getStatus();
    SystemReport leftReport = left.getStatus();
    return rightReport.status.severity > leftReport.status.severity ? rightReport : leftReport;
  }

  /**
   * Sets the state of the flower ramp
   *
   * @param state the new state of the flower ramp
   */
  public void setState(State state) {
    this.state = state;
    updateStateBehavior();
  }

  /**
   * Gets if the flower ramp is at its target position
   *
   * @return true if at target, false if not at target
   */
  public boolean atTarget() {
    return right.atTarget() && left.atTarget();
  }

  /**
   * Gets the current state of the flower ramp
   *
   * @return the flower ramp's state
   */
  public State getState() {
    return state;
  }

  /** Handles state-specific behavior */
  private void updateStateBehavior() {
    switch (state) {
      case STARTING:
        setPositions(STARTING_POS);
        break;

      case STOWED:
        setPositions(STOWED_POS);
        break;

      case DEPLOYED:
        setPositions(DEPLOYED_POS);
        break;
    }
  }

  /**
   * Sets the position of both servos
   *
   * @param positionDegrees the position to set the servos to
   */
  private void setPositions(double positionDegrees) {
    // todo check inversion; maybe invert in servo firmware?
    right.setPosition(positionDegrees + RIGHT_OFFSET);
    left.setPosition(positionDegrees + LEFT_OFFSET);
  }

  private void updateTelemetry() {
    if (!TELEMETRY_ENABLE) return;
    tlm.addData("Flower ramp state", state);
    tlm.addData("Flower ramp at target", atTarget());
    tlm.addData("Flower ramp status", getStatus());
  }
}
