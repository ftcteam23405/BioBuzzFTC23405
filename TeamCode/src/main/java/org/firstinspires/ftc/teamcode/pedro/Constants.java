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
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("fl");
                c.backLeftName.set("bl");
                c.frontRightName.set("fr");
                c.backRightName.set("br");
                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.manualBrakeMode.set(true);
            }
    );
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                c.xPodOffset.set(3.0076513065127877);
                c.yPodOffset.set(-5.2);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.globalDistanceUnit.set(DistanceUnit.INCH);
                c.offsetUnits.set(DistanceUnit.INCH);
            }
    );
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.46263983372785583);
                Controller secondaryTranslationalForward = Controller.proportional(0.170933056598412);
                Controller primaryTranslationalLateral = Controller.proportional(0.6131905081467037);
                Controller secondaryTranslationalLateral = Controller.proportional(0.226557508008067);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011790366755469652));
                c.brake.set(Controller.proportionalFeedforward(0.010021811742149203));

                c.headingFeedback.set(Controller.proportional(6.443675891432396));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05900430045057054, 0.006291937999494106));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07889221809213785, 0.06950338414211264));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002219093423222247, 0.0016244555523818492));

                c.maxAchievableForwardVelocity.set(74.77558405778151);
                c.maxAchievableStrafeVelocity.set(57.5095580015023);
                c.naturalForwardDeceleration.set(36.89019931032952);
                c.naturalStrafeDeceleration.set(67.98208261313123);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}