package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodetypes.TeleOpProgram;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.OpModeType;
import org.firstinspires.ftc.teamcode.utils.OpModeInfo;

@TeleOp(name = "🟥 TeleOp Red", group = "Comp")
public class TeleOpRed extends OpMode {
  private TeleOpProgram program;

  @Override
  public void init() {
    OpModeInfo.type = OpModeType.TELEOP;
    OpModeInfo.color = AllianceColor.RED;
    // Auto will set if robot is loaded

    program = new TeleOpProgram(hardwareMap, telemetry, gamepad1, gamepad2);
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
  }
}
