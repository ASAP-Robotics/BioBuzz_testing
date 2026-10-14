package org.firstinspires.ftc.teamcode.hardware.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.interfaces.MonitoredSystem;
import org.firstinspires.ftc.teamcode.types.SystemReport;
import org.firstinspires.ftc.teamcode.interfaces.System;

/**
 * Class to control an active intake
 */
public class Intake implements MonitoredSystem, System {
  private enum State {
    IDLE(0),
    ACTIVE(1),
    EJECTING(-1);

    State(double motorPower) {
      this.motorPower = motorPower;
    }

    public final double motorPower;
  }

  private final DcMotorEx motor;
  private State state = State.IDLE;

  public Intake(HardwareMap hwMap, String motorName) {
    motor = hwMap.get(DcMotorEx.class, motorName);
  }

  @Override
  public void init() {

  }

  @Override
  public void initLoop() {

  }

  @Override
  public void start() {

  }

  @Override
  public void loop() {
    motor.setPower(state.motorPower);
  }

  @Override
  public void stop() {

  }

  @Override
  public SystemReport getStatus() {
    return null;
  }

  public void setState(State state) {
    this.state = state;
  }
}
