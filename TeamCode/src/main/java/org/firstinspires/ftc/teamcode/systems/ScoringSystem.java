/** Copyright 2026 ASAP Robotics (FTC Team 22029). See LICENCE and NOTICE files for more details. */

package org.firstinspires.ftc.teamcode.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
 * Class to control intake, transfer, turret, flywheel, etc. (scoring-related subsystems)
 */
@Configurable
public class ScoringSystem implements System {
  public enum State {
    INTAKING,
    EJECTING,
    FULL,
    SHOT_QUEUED,
    SHOOTING
  }

  public enum Basket {
    FAR,
    CLOSE
  }

  /** If telemetry should be updated each loop */
  public static boolean TELEMETRY_ENABLE = false;

  /** How long to shoot for, in milliseconds */
  public static double SHOOTING_TIME_MS = 2500.0;

  /** How long to eject for, in milliseconds */
  public static double EJECTING_TIME_MS = 1000.0;

  /** The position of the turret when not tracking the target */
  public static double TURRET_REST_POSITION_DEGREES = 0.0; // todo tune

  /** The RPM of the flywheel when not ready to shoot or shooting */
  public static double FLYWHEEL_IDLE_RPM = 500.0; // todo tune

  private final Intake intake;
  private final Transfer transfer;
  private final Flywheel flywheel;
  private final Turret turret;
  private final AimingCalculator aimingCalculator = new AimingCalculator();
  private final LookupTable lookupTable = new LookupTable(LookupTableContents.TABLE_CONTENTS);
  private final ElapsedTime stateTime;
  private final TelemetryManager tlm;

  private State state = State.INTAKING;
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
    this.stateTime = new ElapsedTime();
    this.tlm = PanelsTelemetry.INSTANCE.getTelemetry();
  }

  @Override
  public void init() {
    intake.init();
    transfer.init();
    flywheel.init();
    turret.init();

    setBasket(Basket.FAR);
    setState(OpModeInfo.robotLoaded ? State.FULL : State.INTAKING);
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

    updateStateLogic();
    updateTelemetry();
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

  /**
   * Ejects both the front intake and the transfer
   */
  public void eject() {
    setState(State.EJECTING);
  }

  /**
   * Sets the robot as "full"; disables the intake and preps for shooting
   */
  public void setFull() {
    if (state != State.INTAKING) return;

    setState(State.FULL);
  }

  /**
   * Shoots the robot's payload of pollen
   */
  public void shoot() {
    if (state == State.SHOOTING) return; // don't cancel active shots

    setState(State.SHOT_QUEUED);
  }

  /**
   * Gets the state of the scoring system
   * @return the current state
   */
  public State getState() {
    return state;
  }

  /**
   * Updates logic to do with the current state
   */
  private void updateStateLogic() {
    switch (state) {
      case INTAKING:
        intake.setState(Intake.State.INTAKING);
        transfer.setState(Transfer.State.INTAKING);
        aimRest();
        break;

      case EJECTING:
        intake.setState(Intake.State.EJECTING);
        transfer.setState(Transfer.State.EJECTING);
        aimRest();
        if (stateTime.milliseconds() > EJECTING_TIME_MS) {
          setState(State.INTAKING);
        }
        break;

      case FULL:
        intake.setState(Intake.State.DISABLED);
        transfer.setState(Transfer.State.DISABLED);
        aimAtTarget();
        break;

      case SHOT_QUEUED:
        intake.setState(Intake.State.DISABLED);
        transfer.setState(Transfer.State.DISABLED);
        aimAtTarget();
        if (locked()) {
          setState(State.SHOOTING);
        }
        break;

      case SHOOTING:
        intake.setState(Intake.State.INTAKING);
        transfer.setState(Transfer.State.SHOOTING);
        aimAtTarget();
        if (stateTime.milliseconds() > SHOOTING_TIME_MS) {
          setState(State.INTAKING);
        }
        break;
    }
  }

  /**
   * Updates telemetry data from scoring system
   */
  private void updateTelemetry() {
    if (!TELEMETRY_ENABLE) return;
    tlm.addData("Scoring system state", state);
    tlm.addData("Scoring system basket", basket);
  }

  /**
   * Sets the state of the scoring system and resets the state time timer if the state is new
   * @param state the (new) state of the scoring system
   */
  private void setState(State state) {
    if (state == null) {
      throw new IllegalArgumentException("State must not be null");
    }

    if (this.state == state) return;

    this.state = state;
    stateTime.reset();
  }

  /**
   * Gets if all shooting / aiming systems are ready to shoot
   *
   * @return true if locked, false if not locked
   */
  private boolean locked() {
    return flywheel.isAtTargetRpm() && turret.atTarget();
  }

  /**
   * Handles aiming at the target
   */
  private void aimAtTarget() {
    turret.setPosition(
        aimingCalculator.getHeading() - aimingCalculator.getPosition().heading(),
        state != State.SHOOTING
    );
    LookupTable.Entry entry = lookupTable.getKeyInterpolate(aimingCalculator.getDistance());
    flywheel.setTargetRpm(entry.value1);
  }

  /**
   * Handles aiming at rest, when not aiming at the target
   */
  private void aimRest() {
    turret.setPosition(TURRET_REST_POSITION_DEGREES, true);
    flywheel.setTargetRpm(FLYWHEEL_IDLE_RPM);
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
