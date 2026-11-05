package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Configurable
@TeleOp
public class ServoPlayground extends OpMode {
  public static double TARGET = 0.0;

  TelemetryManager tlm = PanelsTelemetry.INSTANCE.getTelemetry();

  ServoImplEx servo;


  @Override
  public void init() {
    servo = hardwareMap.get(ServoImplEx.class, "TestServo");
  }

  @Override
  public void loop() {
    if (gamepad1.rightBumperWasPressed()) {
      servo.setPwmEnable();
    } else if (gamepad1.leftBumperWasPressed()) {
      servo.setPwmDisable();
    }

    servo.setPosition(TARGET);

    tlm.addData("Target", TARGET);
    tlm.addData("Enabled", servo.isPwmEnabled());
    tlm.update();
  }
}
