package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Panels tuning test program", group = "Testing")
@Configurable
public class PanelsTest extends OpMode {
  public static int varA = 0;
  public static double varB = 0.0;
  public static boolean varC = false;

  TelemetryManager tlm = PanelsTelemetry.INSTANCE.getTelemetry();

  @Override
  public void init() {

  }

  @Override
  public void loop() {
    tlm.addData("Internal A (int)", varA);
    tlm.addData("Internal B (double)", varB);
    tlm.addData("Internal C (boolean)", varC);
    tlm.addData("Internal success", varA != 0 || varB != 0.0 || varC);
    tlm.addData("External A (int)", PanelsTestConfig.varA);
    tlm.addData("External B (double)", PanelsTestConfig.varB);
    tlm.addData("External C (boolean)", PanelsTestConfig.varC);
    tlm.addData("External success", PanelsTestConfig.varA != 0 || PanelsTestConfig.varB != 0.0 || PanelsTestConfig.varC);
    tlm.update();
  }
}
