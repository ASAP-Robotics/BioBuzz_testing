package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.types.OpModeType;
import org.firstinspires.ftc.teamcode.utils.OpModeInfo;

@TeleOp(name = "Flywheel tuner", group = "Tuning")
@Configurable
public class FlywheelTuner extends OpMode {
  public static double RPM = 0.0;
  public static Intake.State intakeState = Intake.State.DISABLED;
  public static Transfer.State transferState = Transfer.State.DISABLED;

  Flywheel flywheel;
  Intake intake;
  Transfer transfer;
  TelemetryManager tlm;

  @Override
  public void init() {
    OpModeInfo.color = AllianceColor.RED;
    OpModeInfo.type = OpModeType.TELEOP;
    OpModeInfo.robotLoaded = false;

    this.flywheel = new Flywheel(hardwareMap, "Flywheel");
    this.intake = new Intake(hardwareMap, "Intake");
    this.transfer = new Transfer(hardwareMap, "Transfer", "TransferSensor", "Blocker", "BlockerEncoder");
    this.tlm = PanelsTelemetry.INSTANCE.getTelemetry();

    flywheel.init();
    intake.init();
    transfer.init();

    Flywheel.TUNING = true;
    Intake.TELEMETRY_ENABLE = true;
    Transfer.TELEMETRY_ENABLE = true;
  }

  @Override
  public void init_loop() {
    flywheel.initLoop();
    intake.initLoop();
    transfer.initLoop();
  }

  @Override
  public void start() {
    flywheel.start();
    intake.start();
    transfer.start();
  }

  @Override
  public void loop() {
    flywheel.setTargetRpm(RPM);
    intake.setState(intakeState);
    transfer.setState(transferState);

    flywheel.loop();
    intake.loop();
    transfer.loop();

    tlm.update();
  }

  @Override
  public void stop() {
    flywheel.stop();
    intake.stop();
    transfer.stop();
  }
}
