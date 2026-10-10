package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;
import java.util.Map;

// apriltag + shooter tester, makes the follower, limelight, and shooter itself instead of using the Robot class
// during init: dpad left = blue, dpad right = red
// a = spin up (tracks distance to goal), x = spin up to fixedVelocity, b = spin down, y = reset distance filter
// set startX / startY / startHeadingDeg to where the robot actually starts, or the odometry distance is wrong
// tune Limelight.distanceOffset / maxStalenessMs from the dashboard, and the filter with GoalDistanceFilterTuner
@Configurable
@TeleOp
public class AprilTagLimelightTest extends OpMode {
    public static double startX = 72, startY = 24, startHeadingDeg = 90;
    public static double fixedVelocity = 1300;

    private Follower follower;
    private Limelight limelight;
    private Shooter shooter;
    private Alliance alliance = Alliance.BLUE;

    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadLeftWasPressed()) alliance = Alliance.BLUE;
        if (gamepad1.dpadRightWasPressed()) alliance = Alliance.RED;

        telemetry.addData("Alliance", alliance + " (dpad left/right to change)");
        telemetry.addData("Start pose", "(%.1f, %.1f) %.0f deg", startX, startY, startHeadingDeg);
        telemetry.update();
    }

    @Override
    public void start() {
        // made here instead of init() so the alliance picked during init sets the tag pipeline
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
        limelight = new Limelight(hardwareMap, alliance);
        limelight.switchToShootPipeline();
        shooter = new Shooter(hardwareMap);
        loopTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.pose();

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, pose.heading()
        );
        follower.manual(powers);

        if (gamepad1.aWasPressed()) shooter.startTracking();
        if (gamepad1.xWasPressed()) shooter.setTargetVelocity(fixedVelocity);
        if (gamepad1.bWasPressed()) shooter.stop();
        if (gamepad1.yWasPressed()) shooter.resetFilter();

        // one read of the limelight per loop, everything below just uses what it cached
        limelight.periodic();

        // NaN unless a new tag frame came in this loop, so the filter corrects once per frame
        if (shooter.isTracking())
            shooter.setVelocityFromDistance(pose, alliance, limelight.freshDistanceToGoal());
        shooter.periodic();

        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Loop time", "%.1f Hz", 1000 / loopMs);
        telemetry.addLine();

        addShooterTelemetry(pose);
        telemetry.addLine();
        addTagTelemetry();

        telemetry.update();
    }

    private void addShooterTelemetry(Pose pose) {
        telemetry.addLine("--- Shooter ---");
        telemetry.addData("Mode", shooter.isTracking() ? "tracking distance" : "fixed velocity");
        telemetry.addData("Target velocity", "%.0f", shooter.getTargetVelocity());
        telemetry.addData("Actual velocity", "%.0f", shooter.getVelocity());
        telemetry.addData("At target", shooter.atTarget());

        Pose goal = Shooter.closestGoal(pose, alliance);
        telemetry.addData("Closest goal", "(%.1f, %.1f)", goal.x(), goal.y());

        // odometry / limelight / filtered side by side, to check distanceOffset and the filter
        // (odometry and limelight should read about the same when the start pose is right)
        // only updates while tracking
        telemetry.addData("Odometry distance", "%.1f in", shooter.getOdometryDistance());
        double tag = shooter.getLimelightDistance();
        telemetry.addData("Limelight distance", Double.isNaN(tag) ? "no tag" : String.format("%.1f in", tag));
        telemetry.addData("Filtered distance", "%.1f in", shooter.getGoalDistance());
        telemetry.addData("Filter dx (pedro)", "%.2f in", shooter.getFilterDx());
        telemetry.addData("Filter rejects", shooter.getFilterRejects());
        telemetry.addData("Filter corrections", shooter.getFilterCorrections());
        telemetry.addData("Table velocity", "%.0f", Shooter.velocityFor(shooter.getGoalDistance()));
    }

    private void addTagTelemetry() {
        telemetry.addLine("--- Limelight ---");
        telemetry.addData("Pipeline", limelight.getPipleline());

        List<Integer> ids = limelight.visibleTagIds();
        if (ids.isEmpty()) {
            telemetry.addLine("No fresh tag detection");
            return;
        }

        telemetry.addData("Visible tags", ids);
        telemetry.addData("New frame this loop", limelight.hasNewTagFrame());
        telemetry.addData("Tag frames", limelight.getTagFrames());
        telemetry.addData("Distance to goal (in)", "%.1f", limelight.distanceToGoal());
        telemetry.addData("Angle to goal center (deg)", "%.2f", limelight.angle());

        // all the tags on one cell should read the same distance, if they don't fix that tag's offset
        for (Map.Entry<Integer, Double> tag : limelight.tagDistances().entrySet()) {
            telemetry.addData("Tag " + tag.getKey() + " -> goal center",
                    "%.1f in (offset %.1f)", tag.getValue(), Limelight.goalOffset(tag.getKey()));
        }
    }

    @Override
    public void stop() {
        // null if stopped during init
        if (shooter != null) shooter.stop();
        if (limelight != null) limelight.stop();
    }
}
