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
                Controller primaryTranslationalForward = Controller.proportional(0.3822055536910838);
                Controller secondaryTranslationalForward = Controller.proportional(0.14121473936837048);
                Controller primaryTranslationalLateral = Controller.proportional(0.5721226020743553);
                Controller secondaryTranslationalLateral = Controller.proportional(0.21138401406899482);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013450579708098764));
                c.brake.set(Controller.proportionalFeedforward(0.011432992751883949));

                c.headingFeedback.set(Controller.proportional(5.940176621133779));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06288156606455084, 0.00614914541431594));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11397317229117948, 0.06953667318403027));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0010559732273952725, 0.0017585751967231008));

                c.maxAchievableForwardVelocity.set(73.02688314746864);
                c.maxAchievableStrafeVelocity.set(57.21912543933979);
                c.naturalForwardDeceleration.set(32.566614657350435);
                c.naturalStrafeDeceleration.set(67.13177758689022);
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