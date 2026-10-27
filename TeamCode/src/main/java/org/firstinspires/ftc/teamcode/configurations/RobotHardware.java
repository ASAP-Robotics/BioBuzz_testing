/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.configurations;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.indicators.RGBIndicator;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.interfaces.System;

/**
 * Class to encapsulate, expose, and set up the main robot's hardware
 */
public class RobotHardware implements System {
  public final Intake intake;
  public final Flywheel flywheel;
  public final RGBIndicator indicator;
  // more will be added later

  public RobotHardware(HardwareMap hwMap) {
    this.intake = new Intake(hwMap, "IntakeMotor");
    this.flywheel = new Flywheel(hwMap, "FlywheelMotor");
    this.indicator = new RGBIndicator(hwMap, "Indicator1");
  }

  @Override
  public void init() {
    intake.init();
    flywheel.init();
    indicator.init();
  }

  @Override
  public void initLoop() {
    intake.initLoop();
    flywheel.initLoop();
    indicator.initLoop();
  }

  @Override
  public void start() {
    intake.start();
    flywheel.start();
    indicator.start();
  }

  @Override
  public void loop() {
    intake.loop();
    flywheel.loop();
    indicator.loop();
  }

  @Override
  public void stop() {
    intake.stop();
    flywheel.stop();
    indicator.stop();
  }
}
