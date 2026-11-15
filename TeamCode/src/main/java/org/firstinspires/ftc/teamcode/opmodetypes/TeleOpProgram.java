package org.firstinspires.ftc.teamcode.opmodetypes;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class to contain TeleOp program behavior
 */
public class TeleOpProgram extends CommonProgram {
  private final Gamepad gamepad1, gamepad2;

  public TeleOpProgram(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
    super(hwMap, telemetry);
    this.gamepad1 = gamepad1;
    this.gamepad2 = gamepad2;
  }

  @Override
  public void loop() {
    updateDriverControls();

    super.loop();
  }

  /**
   * Handles driver controls
   */
  private void updateDriverControls() {
    if (gamepad1.leftBumperWasPressed()) { // set full
      robot.scoringSystem.setFull();
    }
    if (gamepad1.rightBumperWasPressed()) { // shoot
      robot.scoringSystem.shoot();
    }
    if (gamepad1.aWasPressed()) { // switch target
      robot.scoringSystem.switchTarget();
    }
    if (gamepad1.bWasPressed()) { // eject
      robot.scoringSystem.eject();
    }

    double forward = -gamepad1.left_stick_y;
    double lateral = gamepad1.left_stick_x;
    double turn = gamepad1.right_stick_x;

    robot.follower.manual(forward, lateral, turn); // set driving values
  }
}
