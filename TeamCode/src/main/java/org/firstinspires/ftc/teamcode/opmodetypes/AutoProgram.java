/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.opmodetypes;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.paths.AutoPathsRed;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.utils.RobotPose;

public class AutoProgram extends CommonProgram {
  private final Command pathing;

  public AutoProgram(HardwareMap hwMap, Telemetry telemetry, Pose start, Command pathing) {
    super(hwMap, telemetry);
    this.pathing = pathing;
    AutoPathsRed.setFollower(Constants.create(hwMap));
    RobotPose.robotPose = start;
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
