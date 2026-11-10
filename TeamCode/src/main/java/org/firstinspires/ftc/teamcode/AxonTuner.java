/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.servos.Axon;

@Configurable
@TeleOp(name = "Axon Tuner", group = "Tuning")
public class AxonTuner extends OpMode {
  public static double TARGET_DEGREES;

  Axon axon;
  TelemetryManager tlm;

  @Override
  public void init() {
    axon = new Axon(Axon.Type.MAX, hardwareMap, "TestServo", "TestServoEncoder");
    Axon.TELEMETRY_ENABLE = true;
    axon.init();
    tlm = PanelsTelemetry.INSTANCE.getTelemetry();
    TARGET_DEGREES = 360.0;
  }

  @Override
  public void init_loop() {
    axon.initLoop();
    tlm.update();
  }

  @Override
  public void start() {
    axon.start();
  }

  @Override
  public void loop() {
    if (gamepad1.dpadUpWasPressed()) {
      TARGET_DEGREES += 10;
      if (TARGET_DEGREES > 360) TARGET_DEGREES = 360;
    } else if (gamepad1.dpadDownWasPressed()) {
      TARGET_DEGREES -= 10;
      if (TARGET_DEGREES < 0) TARGET_DEGREES = 0;
    }

    if (gamepad1.dpadLeftWasPressed()) {
      TARGET_DEGREES = 0;
    } else if (gamepad1.dpadRightWasPressed()) {
      TARGET_DEGREES = 360;
    }

    axon.setPosition(TARGET_DEGREES);
    axon.loop();
    tlm.update();
  }

  @Override
  public void stop() {
    axon.stop();
  }
}
