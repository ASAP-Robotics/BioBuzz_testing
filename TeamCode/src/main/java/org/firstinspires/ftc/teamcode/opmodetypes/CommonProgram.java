package org.firstinspires.ftc.teamcode.opmodetypes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configurations.RobotHardware;
import org.firstinspires.ftc.teamcode.interfaces.System;

public abstract class CommonProgram implements System {
  protected final RobotHardware robot;
  protected final Telemetry telemetry;
  protected final TelemetryManager panelsTelemetry;

  public CommonProgram(HardwareMap hwMap, Telemetry telemetry) {
    this.robot = new RobotHardware(hwMap);
    this.telemetry = telemetry;
    this.panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
  }

  @Override
  public void init() {
    robot.init();
    updateTelemetry();
  }

  @Override
  public void initLoop() {
    robot.initLoop();
    updateTelemetry();
  }

  @Override
  public void start() {
    robot.start();
    updateTelemetry();
  }

  @Override
  public void loop() {
    robot.scoringSystem.setPosition(robot.follower.pose()); // update current position
    robot.loop();
    robot.follower.update();
    updateTelemetry();
  }

  @Override
  public void stop() {
    robot.stop();
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
