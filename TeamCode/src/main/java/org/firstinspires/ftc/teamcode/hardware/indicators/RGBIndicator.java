/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.hardware.indicators;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.utils.Follower;

/** Simple class to control a GoBILDA "RGB Indicator light (PWM controlled)" */
public class RGBIndicator {
  public enum Color {
    OFF(0.0),
    RED(0.288),
    ORANGE(0.333),
    YELLOW(0.388),
    SAGE(0.444),
    GREEN(0.500),
    AZURE(0.555),
    BLUE(0.611),
    INDIGO(0.666),
    VIOLET(0.722),
    WHITE(1.0);

    public final double num;

    Color(double num) {
      this.num = num;
    }
  }

  private static final double UPDATE_TOLERANCE = 0.01;
  private final Servo led; // light is controlled by servo PWM control
  private final Follower follower = new Follower(0, 0, 0, 1.0);
  private Color color = null;
  private double lastSetValue = Double.NEGATIVE_INFINITY;
  private boolean atColor = false;

  /**
   * Creates a new RGBIndicator object
   *
   * @param hardwareMap the robot's hardware map
   * @param deviceName the configured name of the indicator light
   */
  public RGBIndicator(HardwareMap hardwareMap, String deviceName) {
    this.led = hardwareMap.get(Servo.class, deviceName);
  }

  /**
   * Creates a new RGBIndicator object
   *
   * @param led a Servo object that represents the indicator light
   * @implNote since GoBILDAs RGB indicators are controlled by a PWM output meant for servos, we use
   *     a Servo object to control the light (hence the type passed here)
   */
  public RGBIndicator(Servo led) {
    this.led = led;
  }

  /**
   * Update the indicator light
   *
   * @apiNote this should be called every loop; the color of the light won't change if this method
   *     isn't called
   */
  public void update() {
    if (atColor) return;
    double value = follower.getValue();

    if (Math.abs(value - lastSetValue) > UPDATE_TOLERANCE) {
      led.setPosition(value);
      lastSetValue = value;
    }

    if (follower.isAtTarget()) atColor = true;
  }

  /**
   * Sets the target color of the light (the color the light will fade to)
   *
   * @param color the color to set the light to
   */
  public void setColor(Color color) {
    if (color == this.color) return;
    this.color = color;
    atColor = false;
    follower.setTarget(color.num);
  }

  /**
   * Gets the color that the light is fading towards
   *
   * @return the target color of the light
   */
  public Color getTargetColor() {
    return color;
  }

  /**
   * Gets the color closest to the currently displayed "color" of the light
   *
   * @return the color that the currently displayed color is closest to
   */
  public Color getColor() {
    // untested
    Color bestColor = Color.OFF;
    double bestDiff = Double.POSITIVE_INFINITY;
    for (Color i : Color.values()) {
      double diff = Math.abs(i.num - lastSetValue);
      if (diff < bestDiff) {
        bestDiff = diff;
        bestColor = i;
      }
    }

    return bestColor;
  }
}
