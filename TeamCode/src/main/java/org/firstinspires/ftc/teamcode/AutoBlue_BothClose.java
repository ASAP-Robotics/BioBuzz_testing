/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.opmodetypes.AutoProgram;
import org.firstinspires.ftc.teamcode.paths.AutoPaths;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.OpModeType;
import org.firstinspires.ftc.teamcode.utils.OpModeInfo;

@Autonomous(name = "🟦 Auto Blue Both Close", group = "Comp")
public class AutoBlue_BothClose extends OpMode {
  private AutoProgram program;

  @Override
  public void init() {
    OpModeInfo.type = OpModeType.AUTO;
    OpModeInfo.color = AllianceColor.BLUE;
    OpModeInfo.robotLoaded = true;

    program = new AutoProgram(hardwareMap, telemetry, AutoProgram.AutoType.BOTH_CLOSE);
    program.init();
  }

  @Override
  public void init_loop() {
    program.initLoop();
  }

  @Override
  public void start() {
    program.start();
  }

  @Override
  public void loop() {
    program.loop();
  }

  @Override
  public void stop() {
    program.stop();
    OpModeInfo.robotLoaded = false;
  }
}
