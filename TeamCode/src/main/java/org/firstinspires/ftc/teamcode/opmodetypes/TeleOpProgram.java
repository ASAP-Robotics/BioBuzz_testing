/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.opmodetypes;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.FlowerRamp;

/**
 * Class to contain TeleOp program behavior
 */
public class TeleOpProgram extends CommonProgram {
  private final Gamepad gamepad1, gamepad2;

  public TeleOpProgram(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
    super(hwMap, telemetry);
    if (gamepad1 == null || gamepad2 == null) {
      throw new IllegalArgumentException("Gamepads must not be null");
    }
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
    if (gamepad1.bWasPressed()) { // eject all
      robot.scoringSystem.eject();
    }
    if (gamepad1.xWasPressed()) { // toggle flower ramp
      robot.flowerRamp.setState(
          robot.flowerRamp.getState() == FlowerRamp.State.STOWED ?
              FlowerRamp.State.DEPLOYED : FlowerRamp.State.STOWED
      );
    }
    if (gamepad1.yWasPressed()) { // eject intake
      robot.scoringSystem.ejectIntake();
    }
    if (gamepad2.aWasPressed()) { // shooting backup deploy blocker
      robot.scoringSystem.shootBackup();
    }
    if (gamepad2.bWasPressed()) { // switch aiming mode
      robot.scoringSystem.switchAimingMode();
    }

    double forward = -gamepad1.right_stick_y;
    double lateral = -gamepad1.right_stick_x;
    double turn = -gamepad1.left_stick_x;

    robot.follower.manual(forward, lateral, turn); // set driving values
  }
}
