/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.opmodetypes;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.paths.AutoPaths;
import org.firstinspires.ftc.teamcode.utils.RobotPose;

public class AutoProgram extends CommonProgram {
  private final Command pathing;

  /**
   * Constructor
   *
   * @param hwMap hardware map
   * @param telemetry telemetry
   *
   * @apiNote OpMode info MUST be filled out prior to calling this!
   */
  public AutoProgram(HardwareMap hwMap, Telemetry telemetry) {
    super(hwMap, telemetry);
    AutoPaths paths = new AutoPaths(robot);
    this.pathing = paths.getCloseAuto();
    RobotPose.robotPose = paths.getCloseAutoStart();
  }

  @Override
  public void init() {
    super.init();

    Scheduler.reset();
  }

  @Override
  public void initLoop() {
    super.initLoop();
  }

  @Override
  public void start() {
    super.start();

    Scheduler.schedule(pathing);
  }

  @Override
  public void loop() {
    Scheduler.execute();

    super.loop();
  }

  @Override
  public void stop() {
    super.stop();
  }
}
