/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.configurations;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.indicators.RGBIndicator;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.systems.ScoringSystem;

/** Class to encapsulate, expose, and set up the main robot's hardware */
public class RobotHardware implements System {
  public final ScoringSystem scoringSystem;
  public final Follower follower;

  // todo add flower intake

  public RobotHardware(HardwareMap hwMap) {
    this.scoringSystem =
        new ScoringSystem(
            hwMap,
            "Intake",
            "Transfer",
            "TransferSensor",
            "Blocker",
            "BlockerEncoder",
            "Flywheel",
            "Turret",
            "TurretSensor"
        );

    this.follower = Constants.create(hwMap);
  }

  @Override
  public void init() {
    scoringSystem.init();
  }

  @Override
  public void initLoop() {
    scoringSystem.initLoop();
  }

  @Override
  public void start() {
    scoringSystem.start();
  }

  @Override
  public void loop() {
    scoringSystem.loop();
  }

  @Override
  public void stop() {
    scoringSystem.stop();
  }
}
