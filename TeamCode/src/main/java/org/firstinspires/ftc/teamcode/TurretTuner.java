/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Turret;

@TeleOp(name = "Turret tuner", group = "Tuning")
@Configurable
public class TurretTuner extends OpMode {
  private Turret turret;

  private TelemetryManager tlm;

  public static double POSITION = 0.0;
  public static boolean HOME = false;

  @Override
  public void init() {
    this.turret = new Turret(hardwareMap, "Turret", "TurretSensor");
    this.tlm = PanelsTelemetry.INSTANCE.getTelemetry();

    turret.init();
  }

  public void initLoop() {
    turret.initLoop();
  }

  @Override
  public void start() {
    turret.start();
  }

  @Override
  public void loop() {
    turret.setPosition(POSITION, true);

    if (HOME) {
      turret.home();
      HOME = false;
    }

    turret.loop();

    tlm.update();
  }

  public void stop() {
    turret.stop();
  }
}
