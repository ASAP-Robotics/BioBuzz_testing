/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.configurations;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.FlowerRamp;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.systems.ScoringSystem;

/** Class to encapsulate, expose, and set up the main robot's hardware */
public class RobotHardware implements System {
  public final ScoringSystem scoringSystem;
  public final FlowerRamp flowerRamp;
  public final Follower follower;

  public RobotHardware(HardwareMap hwMap, Telemetry telemetry) {
    this.scoringSystem =
        new ScoringSystem(
            hwMap,
            telemetry,
            "Intake",
            "Transfer",
            "TransferSensor",
            "Blocker",
            "BlockerEncoder",
            "Flywheel",
            "Turret",
            "TurretSensor");

    this.flowerRamp =
        new FlowerRamp(hwMap, "RampRight", "RampRightEncoder", "RampLeft", "RampLeftEncoder");

    this.follower = Constants.create(hwMap);
  }

  @Override
  public void init() {
    scoringSystem.init();
    flowerRamp.init();
  }

  @Override
  public void initLoop() {
    scoringSystem.initLoop();
    flowerRamp.initLoop();
  }

  @Override
  public void start() {
    scoringSystem.start();
    flowerRamp.start();
  }

  @Override
  public void loop() {
    scoringSystem.loop();
    flowerRamp.loop();
  }

  @Override
  public void stop() {
    scoringSystem.stop();
    flowerRamp.stop();
  }
}
