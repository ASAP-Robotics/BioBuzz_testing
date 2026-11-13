package org.firstinspires.ftc.teamcode.systems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.BasketPositions;
import org.firstinspires.ftc.teamcode.constants.LookupTableContents;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.hardware.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.interfaces.System;
import org.firstinspires.ftc.teamcode.types.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.AimingCalculator;
import org.firstinspires.ftc.teamcode.utils.LookupTable;
import org.firstinspires.ftc.teamcode.utils.OpModeInfo;

// todo WIP

/**
 * Class to control intake, turret, and flywheel (scoring-related subsystems)
 */
public class ScoringSystem implements System {
  public enum Basket {
    FAR,
    CLOSE
  }

  private final Intake intake;
  private final Transfer transfer;
  private final Flywheel flywheel;
  private final Turret turret;
  private final AimingCalculator aimingCalculator = new AimingCalculator();
  private final LookupTable lookupTable = new LookupTable(LookupTableContents.TABLE_CONTENTS);

  private Basket basket = Basket.FAR;

  public ScoringSystem(HardwareMap hwMap,
                       String intakeMotorName,
                       String transferMotorName,
                       String transferSensorName,
                       String transferServoName,
                       String transferServoEncoderName,
                       String flywheelMotorName,
                       String turretMotorName,
                       String turretBreakBeamName) {
    this.intake = new Intake(hwMap, intakeMotorName);
    this.transfer = new Transfer(
        hwMap,
        transferMotorName,
        transferSensorName,
        transferServoName,
        transferServoEncoderName
    );
    this.flywheel = new Flywheel(hwMap, flywheelMotorName);
    this.turret = new Turret(hwMap, turretMotorName, turretBreakBeamName);
  }

  @Override
  public void init() {
    intake.init();
    transfer.init();
    flywheel.init();
    turret.init();

    setBasket(Basket.FAR);
  }

  @Override
  public void initLoop() {
    intake.initLoop();
    transfer.initLoop();
    flywheel.initLoop();
    turret.initLoop();
  }

  @Override
  public void start() {
    intake.start();
    transfer.start();
    flywheel.start();
    turret.start();
  }

  @Override
  public void loop() {
    intake.loop();
    transfer.loop();
    flywheel.loop();
    turret.loop();

    // todo logic
  }

  @Override
  public void stop() {
    intake.stop();
    transfer.stop();
    flywheel.stop();
    turret.stop();
  }

  /**
   * Sets the current position of the robot on the field
   *
   * @param robotPosition the robot's current position
   */
  public void setPosition(Pose robotPosition) {
    aimingCalculator.setPosition(robotPosition);
  }

  /**
   * Switches from aiming at one basket to the other basket
   */
  public void switchTarget() {
    setBasket(basket == Basket.FAR ? Basket.CLOSE : Basket.FAR);
  }

  private void setBasket(Basket basket) {
    switch (basket) {
      case FAR:
        if (OpModeInfo.color == AllianceColor.RED) {
          aimingCalculator.setTarget(BasketPositions.RED_BASKET_FAR_POSITION);
        } else {
          aimingCalculator.setTarget(BasketPositions.BLUE_BASKET_FAR_POSITION);
        }
        break;

      case CLOSE:
        if (OpModeInfo.color == AllianceColor.RED) {
          aimingCalculator.setTarget(BasketPositions.RED_BASKET_CLOSE_POSITION);
        } else {
          aimingCalculator.setTarget(BasketPositions.BLUE_BASKET_CLOSE_POSITION);
        }
        break;
    }

    this.basket = basket;
  }
}
