/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.opmodetypes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.LoopProgram;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class AutoProgram extends CommonProgram {

  private final Follower follower;

  public AutoProgram(HardwareMap hwMap, Telemetry telemetry) {
    super(hwMap, telemetry);
    this.follower = Constants.create(hwMap);
  }

  // todo


  @Override
  public void init() {
    super.init();
  }

  @Override
  public void initLoop() {
    super.initLoop();
  }

  @Override
  public void start() {
    super.start();

    Command testLooper = new LoopProgram(this);

    // todo add commands, etc., and looping command
  }

  @Override
  public void loop() {
    // never used directly; start() never exits
    super.loop();
  }

  @Override
  public void stop() {
    super.stop();
  }
}
