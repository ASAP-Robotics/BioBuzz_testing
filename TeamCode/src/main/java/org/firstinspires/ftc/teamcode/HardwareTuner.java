/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.OpModeType;
import org.firstinspires.ftc.teamcode.utils.LoopTimeTracker;
import org.firstinspires.ftc.teamcode.utils.OpModeInfo;

@TeleOp(name = "Hardware tuner", group = "Tuning")
@Configurable
public class HardwareTuner extends OpMode {
  private Turret turret;
  private Flywheel flywheel;
  Intake intake;
  Transfer transfer;
  private LoopTimeTracker loopTimeTracker;

  private TelemetryManager tlm;

  public static double TURRET_POSITION = 0.0;
  public static boolean HOME_TURRET = false;
  public static boolean ALLOW_FLIPPING = true;
  public static double FLYWHEEL_RPM = 0.0;
  public static Intake.State INTAKE_STATE = Intake.State.DISABLED;
  public static Transfer.State TRANSFER_STATE = Transfer.State.DISABLED;

  @Override
  public void init() {
    OpModeInfo.color = AllianceColor.RED;
    OpModeInfo.type = OpModeType.TELEOP;
    OpModeInfo.robotLoaded = false;

    this.turret = new Turret(hardwareMap, "Turret", "TurretSensor");
    this.flywheel = new Flywheel(hardwareMap, "Flywheel");
    this.intake = new Intake(hardwareMap, "Intake");
    this.transfer =
        new Transfer(hardwareMap, "Transfer", "TransferSensor", "Blocker", "BlockerEncoder");
    this.loopTimeTracker = new LoopTimeTracker();
    this.tlm = PanelsTelemetry.INSTANCE.getTelemetry();

    turret.init();
    flywheel.init();
    intake.init();
    transfer.init();

    Turret.TELEMETRY_ENABLE = true;
    Flywheel.TUNING = true;
    Intake.TELEMETRY_ENABLE = true;
    Transfer.TELEMETRY_ENABLE = true;

    loopTimeTracker.start();
  }

  public void initLoop() {
    turret.initLoop();
    flywheel.initLoop();
    intake.initLoop();
    transfer.initLoop();
    loopTimeTracker.loop();
  }

  @Override
  public void start() {
    turret.start();
    flywheel.start();
    intake.start();
    transfer.start();
    loopTimeTracker.reset();
  }

  @Override
  public void loop() {
    turret.setPosition(TURRET_POSITION, ALLOW_FLIPPING);
    flywheel.setTargetRpm(FLYWHEEL_RPM);
    intake.setState(INTAKE_STATE);
    transfer.setState(TRANSFER_STATE);

    if (HOME_TURRET) {
      turret.home();
      HOME_TURRET = false;
    }

    turret.loop();
    flywheel.loop();
    intake.loop();
    transfer.loop();
    loopTimeTracker.loop();

    tlm.update();
  }

  public void stop() {
    turret.stop();
    turret.stop();
    intake.stop();
    transfer.stop();
  }
}
