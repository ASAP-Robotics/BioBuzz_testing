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
            c.xPodOffset.set(7.0);
            c.yPodOffset.set(0.67);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
          });

  public static ForesightConfig foresightConfig =
      new ForesightConfig(
          c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.4015849370663579);
            Controller secondaryTranslationalForward = Controller.proportional(0.1483749036988735);
            Controller primaryTranslationalLateral = Controller.proportional(0.6487477926916921);
            Controller secondaryTranslationalLateral = Controller.proportional(0.23969497453929886);

            c.forwardTranslational.set(
                Controller.piecewise(secondaryTranslationalForward)
                    .put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(
                Controller.piecewise(secondaryTranslationalLateral)
                    .put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.013572706081808786));
            c.brake.set(Controller.proportionalFeedforward(0.011536800169537467));

            c.headingFeedback.set(Controller.proportional(6.466062282178606));
            c.headingBrakeCoefficients.set(
                Vector2D.cartesian(0.06191123908405446, 0.0042970963587870645));

            c.linearBrakeCoefficients.set(Matrix.diag(0.08203274162344029, 0.08543888119418692));
            c.quadraticBrakeCoefficients.set(
                Matrix.diag(0.0017765013038332552, 0.0012009168072013732));

            c.maxAchievableForwardVelocity.set(74.94564981293682);
            c.maxAchievableStrafeVelocity.set(54.979107485107804);
            c.naturalForwardDeceleration.set(37.311813508470976);
            c.naturalStrafeDeceleration.set(75.2682946158783);
          });

  public static Follower create(HardwareMap h) {
    return new Follower(
        new PinpointLocalizer(h, localizerConfig),
        new Mecanum(h, drivetrainConfig),
        new Foresight(foresightConfig));
  }
}
