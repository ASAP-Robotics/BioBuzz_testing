/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.hardware.sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Class to use a simple break beam sensor
 *
 * @implNote assumes an always-on LED
 */
public class BreakBeam {
  public enum State {
    BROKEN,
    UNBROKEN
  }

  protected static final boolean BROKEN_BEAM_STATE = false; // the state when the beam is broken
  protected final DigitalChannel sensor;

  public BreakBeam(HardwareMap hwMap, String deviceName) {
    this(hwMap.get(DigitalChannel.class, deviceName));
  }

  public BreakBeam(DigitalChannel sensor) {
    this.sensor = sensor;
    this.sensor.setMode(DigitalChannel.Mode.INPUT);
  }

  /**
   * Gets the current state of the sensor's beam
   *
   * @return the state of the beam
   */
  public State getState() {
    if (sensor.getState() == BROKEN_BEAM_STATE) {
      return State.BROKEN;
    } else {
      return State.UNBROKEN;
    }
  }

  /**
   * Gets if the beam is broken
   *
   * @return true if the beam is broken, false otherwise
   */
  public boolean isBroken() {
    return getState() == State.BROKEN;
  }

  /**
   * Gets if the beam is unbroken
   *
   * @return true if the beam is unbroken, false otherwise
   */
  public boolean isUnbroken() {
    return getState() == State.UNBROKEN;
  }
}
