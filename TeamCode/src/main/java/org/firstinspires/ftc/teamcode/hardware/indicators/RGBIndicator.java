/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.hardware.indicators;

import static org.firstinspires.ftc.teamcode.constants.UpdateTolerance.SERVO_UPDATE_TOLERANCE;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.utils.Follower;

/** Simple class to control a GoBILDA "RGB Indicator light (PWM controlled)" */
public class RGBIndicator implements System {
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

  private final Servo led; // light is controlled by PWM control over a servo channel
  private final Follower follower = new Follower(0.0, 0.0, 0, 1.0);
  private Color color = Color.OFF;
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

  @Override
  public void init() {
    update();
  }

  @Override
  public void initLoop() {
    update();
  }

  @Override
  public void start() {
    update();
  }

  @Override
  public void loop() {
    update();
  }

  @Override
  public void stop() {
    update();
  }

  /**
   * Updates the indicator light
   *
   * @apiNote this should be called every loop and every target color change; the color of the light
   *     won't change if this method isn't called
   */
  protected void update() {
    if (atColor) return;
    double value = follower.getValue();
    if (follower.isAtTarget()) atColor = true; // placing this before potentially slow servo writes

    if (Math.abs(value - lastSetValue) > SERVO_UPDATE_TOLERANCE) {
      led.setPosition(value);
      lastSetValue = value;
    }
  }

  /**
   * Sets the display color of the light (the light will immediately display this color).
   *
   * <p>This method is fairly fast (1 hardware write if new color is set), assuming a new (unique)
   * color isn't set each loop
   *
   * @param color the color to set the light to
   */
  public void setColor(Color color) {
    if (color == this.color && atColor) return;
    this.color = color;
    atColor = false; // update() will set this to true, but won't change light unless it is false
    follower.setTarget(color.num);
    follower.setValue(color.num);
    update();
  }

  /**
   * Sets the target color of the light (the color the light will fade to).
   *
   * <p><strong>⚠️ Avoid this method for competition use.</strong> Because fading to a new color
   * requires hardware writes every loop until the target color is reached, using this method can
   * detriment loop times. Use this method only when loop times don't matter, like in testing or
   * example code.
   *
   * @param color the color to set the light to
   * @see setColor
   */
  public void setColorFade(Color color) {
    if (color == this.color) return;
    this.color = color;
    atColor = false;
    follower.setTarget(color.num);
    update();
  }

  /**
   * Gets the color that the light is displaying or fading towards
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
   * @apiNote don't use this method for anything too important; it's kinda slow and could be erratic
   * @see getTargetColor
   */
  public Color getColor() {
    // untested, but should work
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
