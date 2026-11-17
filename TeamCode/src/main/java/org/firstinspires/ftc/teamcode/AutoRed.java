package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmodetypes.AutoProgram;
import org.firstinspires.ftc.teamcode.paths.AutoPathsRed;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.OpModeType;
import org.firstinspires.ftc.teamcode.utils.OpModeInfo;

@Autonomous(name = "🟥 Auto Red Close", group = "Comp")
public class AutoRed extends OpMode {
  private AutoProgram program;

  @Override
  public void init() {
    OpModeInfo.type = OpModeType.AUTO;
    OpModeInfo.color = AllianceColor.RED;
    OpModeInfo.robotLoaded = true;

    program = new AutoProgram(hardwareMap, telemetry, AutoPathsRed.start, AutoPathsRed.RED_CLOSE_AUTO);
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
