/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.opmodetypes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configurations.RobotHardware;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.utils.BulkCachingHelper;
import org.firstinspires.ftc.teamcode.utils.LoopTimeTracker;
import org.firstinspires.ftc.teamcode.utils.RobotPose;

@Configurable
public abstract class CommonProgram implements System {
  protected final BulkCachingHelper bulkCacher;
  protected final RobotHardware robot;
  protected final Telemetry telemetry;
  protected final TelemetryManager panelsTelemetry;
  protected final LoopTimeTracker loopTimeTracker;

  public CommonProgram(HardwareMap hwMap, Telemetry telemetry) {
    if (hwMap == null || telemetry == null) {
      throw new IllegalArgumentException("Hardware map and telemetry must not be null");
    }
    this.bulkCacher = new BulkCachingHelper(hwMap);
    this.robot = new RobotHardware(hwMap);
    this.telemetry = telemetry;
    this.panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    this.loopTimeTracker = new LoopTimeTracker();
  }

  @Override
  public void init() {
    bulkCacher.init();
    robot.init();
    robot.follower.setPose(RobotPose.robotPose); // set robot position
    loopTimeTracker.start();
    updateTelemetry();
  }

  @Override
  public void initLoop() {
    bulkCacher.initLoop();
    robot.initLoop();
    loopTimeTracker.loop();
    updateTelemetry();
  }

  @Override
  public void start() {
    bulkCacher.start();
    robot.start();
    loopTimeTracker.reset();
    updateTelemetry();
  }

  @Override
  public void loop() {
    bulkCacher.loop();
    robot.scoringSystem.setPosition(robot.follower.pose()); // update current position
    robot.loop();
    loopTimeTracker.loop();
    robot.follower.update();
    updateTelemetry();
  }

  @Override
  public void stop() {
    bulkCacher.stop();
    robot.stop();
    RobotPose.robotPose = robot.follower.pose(); // save robot position for future OpModes
    updateTelemetry();
  }

  /**
   * Updates telemetry
   */
  protected void updateTelemetry() {
    telemetry.update();
    panelsTelemetry.update();
  }
}
