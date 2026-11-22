package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
  public static MecanumConfig drivetrainConfig =
      new MecanumConfig(
          c -> {
            c.frontLeftName.set("fl");
            c.frontRightName.set("fr");
            c.backLeftName.set("bl");
            c.backRightName.set("br");
            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
          });

  public static PinpointConfig localizerConfig =
      new PinpointConfig(
          c -> {
            c.name.set("Pinpoint");
            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            c.xPodOffset.set(-7.5);
            c.yPodOffset.set(0.09);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
          });

  public static ForesightConfig foresightConfig =
      new ForesightConfig(
          c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.3257759569544337);
            Controller secondaryTranslationalForward = Controller.proportional(0.1203655112007731);
            Controller primaryTranslationalLateral = Controller.proportional(0.535689985060025);
            Controller secondaryTranslationalLateral = Controller.proportional(0.19792313557965557);

            c.forwardTranslational.set(
                Controller.piecewise(secondaryTranslationalForward)
                    .put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(
                Controller.piecewise(secondaryTranslationalLateral)
                    .put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.013636082087269177));
            c.brake.set(Controller.proportionalFeedforward(0.0115906697741788));

            c.headingFeedback.set(Controller.proportional(6.217505582519955));
            c.headingBrakeCoefficients.set(
                Vector2D.cartesian(0.0569277136503686, 0.005810811230843468));

            c.linearBrakeCoefficients.set(Matrix.diag(0.1250980323836812, 0.06418433846563949));
            c.quadraticBrakeCoefficients.set(
                Matrix.diag(7.346140688120379E-4, 0.0015047552730513094));

            c.maxAchievableForwardVelocity.set(78.0649922282979);
            c.maxAchievableStrafeVelocity.set(58.12030661383527);
            c.naturalForwardDeceleration.set(32.04324766913408);
            c.naturalStrafeDeceleration.set(68.07813651549037);
          });

  public static Follower create(HardwareMap h) {
    return new Follower(
        new PinpointLocalizer(h, localizerConfig),
        new Mecanum(h, drivetrainConfig),
        new Foresight(foresightConfig));
  }
}
