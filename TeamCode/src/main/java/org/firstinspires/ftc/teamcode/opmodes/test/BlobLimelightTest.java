package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.commandbase.vision.PollenDetector;
import org.firstinspires.ftc.teamcode.pedro.Constants;

// blob detection tester, runs through the Limelight subsystem's periodic()
// a = blob pipeline, b = tags pipeline (to check switching back and forth)
@Configurable
@TeleOp
public class BlobLimelightTest extends OpMode {
    private Follower follower;
    private Limelight limelight;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0).withHeading(Math.toRadians(0)));

        limelight = new Limelight(hardwareMap, Alliance.BLUE);
    }

    @Override
    public void start() {
        limelight.switchToBlobPipeline();
    }

    @Override
    public void loop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, follower.pose().heading()
        );
        follower.manual(powers);
        follower.update();

        limelight.periodic();

        telemetry.addData("Pipeline", limelight.getPipleline());
        telemetry.addData("Fresh blob data", limelight.hasFreshBlobData());
        telemetry.addData("Pollen count (smoothed)", limelight.getPollenCount());

        PollenDetector.Ball best = limelight.getBestPollen();
        if (best == null) {
            telemetry.addLine("No pollen");
        } else {
            telemetry.addData("Best tx (deg)", "%.2f", best.tx);
            telemetry.addData("Best ty (deg)", "%.2f", best.ty);
            telemetry.addData("Best radius (px)", "%.1f", best.radiusPx);
            telemetry.addData("Best score", "%.2f", best.score);
        }

        telemetry.addData("Balls this frame", limelight.getBalls().size());
        int i = 0;
        for (PollenDetector.Ball b : limelight.getBalls()) {
            telemetry.addData("Ball " + i++, "%s tx %.1f ty %.1f r %.1f s %.2f",
                    b.isPollen ? "POLLEN" : "OTHER", b.tx, b.ty, b.radiusPx, b.score);
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
