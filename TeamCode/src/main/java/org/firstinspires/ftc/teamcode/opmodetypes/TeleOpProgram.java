/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */
package org.firstinspires.ftc.teamcode.opmodetypes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.FlowerRamp;

/** Class to contain TeleOp program behavior */
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
  public void start() {
    // resetPosition();

    super.start();
  }

  @Override
  public void loop() {
    updateDriverControls();

    super.loop();
  }

  public void resetPosition() {
    robot.follower.setPose(new Pose(9.0, 9.0, Math.PI / 2));
  }

  /** Handles driver controls */
  private void updateDriverControls() {
    updateCommonDriverControls(gamepad1);
    updateCommonDriverControls(gamepad2);

    double forward = -gamepad1.right_stick_y;
    double lateral = -gamepad1.right_stick_x;
    double turn = -gamepad1.left_stick_x;

    robot.follower.manual(forward, lateral, turn); // set driving values
  }

  /** Updates driver controls common for any (both) gamepad(s) */
  private void updateCommonDriverControls(Gamepad gamepad) {
    if (gamepad.right_trigger >= 0.5 && gamepad.left_trigger >= 0.5) { // SHIFT + ALT
      // cancel shot
      robot.scoringSystem.cancelShot();

    } else if (gamepad.right_trigger >= 0.5) { // SHIFT
      if (gamepad.leftBumperWasPressed()) { // cancel set full
        robot.scoringSystem.setNotFull();
      }
      if (gamepad.aWasPressed()) { // shooting backup deploy blocker
        robot.scoringSystem.shootBackup();
      }
      if (gamepad.bWasPressed()) { // switch aiming mode
        robot.scoringSystem.switchAimingMode();
      }
      if (gamepad.xWasPressed()) { // reset position
        resetPosition();
      }
      if (gamepad.yWasPressed()) { // home turret
        robot.scoringSystem.homeTurret();
      }
      // todo turret offsets

    } else if (gamepad.left_trigger >= 0.5) { // ALT

    } else { // NORMAL
      if (gamepad.leftBumperWasPressed()) { // set full
        robot.scoringSystem.setFull();
      }
      if (gamepad.rightBumperWasPressed()) { // shoot
        robot.scoringSystem.shoot();
      }
      if (gamepad.aWasPressed()) { // switch target
        robot.scoringSystem.switchTarget();
      }
      if (gamepad.bWasPressed()) { // eject all
        robot.scoringSystem.eject();
      }
      if (gamepad.xWasPressed()) { // toggle flower ramp
        switch (robot.flowerRamp.getState()) {
          case STARTING:
            robot.flowerRamp.setState(FlowerRamp.State.DEPLOYED);
            break;

          case DEPLOYED:
            robot.flowerRamp.setState(FlowerRamp.State.STOWED);
            break;

          case STOWED:
            robot.flowerRamp.setState(FlowerRamp.State.STARTING);
            break;
        }
      }
      if (gamepad.yWasPressed()) { // eject intake
        robot.scoringSystem.ejectIntake();
      }
    }
  }
}
