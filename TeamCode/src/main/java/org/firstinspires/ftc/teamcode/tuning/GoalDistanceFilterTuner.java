package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.commandbase.util.GoalDistanceFilter;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayDeque;
import java.util.Locale;
import java.util.Map;

// tunes GoalDistanceFilter: the limelight/pedro blend that Shooter shoots off of
// runs the follower, the limelight and the filter itself, no flywheels, so it's safe to drive around with
//
// during init: dpad left = blue, dpad right = red
// driving: left stick translate, right stick turn (field centric)
// dpad up/down    = limelightWeight +/- weightStep
// dpad left/right = Limelight.distanceOffset +/- offsetStep
// a = reset the filter   b = reset the stats   x = put the follower back on the start pose
// y = hold pedro only (weight 0), to watch how fast odometry drifts   back = clear everything
//
// how to tune, in order:
// 1. offsets. these are already measured off the game manual's cluster drawing, so this is a check, not
//    a tune. park in front of a cell so 2+ tags are in view: every tag should report the same distance to
//    goal and SPREAD should sit under about an inch. a big spread means that cell's tag ids run right to
//    left instead of left to right, so flip the argument order for that cell in Limelight.cell(). for one
//    tag that's off on its own, set Limelight.tuneTagId to it, drag Limelight.tuneOffset until it agrees,
//    and paste that number into GOAL_OFFSETS
// 2. distanceOffset. tape measure from the shooter to the goal center, put it in trueDistance, park there,
//    and dpad left/right until the LIMELIGHT error reads about 0
// 3. limelightWeight. stand still and watch the NOISE lines. raw is what the limelight gives you, filtered
//    is what the shooter gets. higher weight = tracks the limelight harder and jumps more, lower = smoother
//    but leans on pedro, which drifts. keep it high (0.8 default) and only come down if the filtered noise
//    is big enough to matter, an inch or two of noise is nothing next to pedro drifting all match
// 4. maxJump. drive around and watch REJECTS. it should sit at 0. if it climbs while the numbers look fine,
//    maxJump is too tight for how far pedro drifts between frames
@Configurable
@Config
@TeleOp
public class GoalDistanceFilterTuner extends OpMode {
    public static double startX = 8, startY = 134, startHeadingDeg = 90; //start in top left corner pedro
    public static double trueDistance = 0;   // tape measured distance to the goal center, 0 = don't score errors
    public static double weightStep = 0.05;
    public static double offsetStep = 0.5;
    public static int statSamples = 50;      // limelight frames kept for the noise/error numbers

    private Follower follower;
    private Limelight limelight;
    private Alliance alliance = Alliance.BLUE;

    private final GoalDistanceFilter filter = new GoalDistanceFilter();
    private boolean pedroOnly = false;
    private double savedWeight = 0;

    private double odometryDistance = 0, filtered = 0;
    private double lastLimelight = Double.NaN;

    // one sample per new limelight frame: {raw limelight, filtered, odometry}
    private final ArrayDeque<double[]> samples = new ArrayDeque<>();
    private int frames = 0;
    private final ElapsedTime frameTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double frameHz = 0;

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
        telemetry.addLine("set the start pose to where the robot actually is, the odometry distance needs it");
        telemetry.update();
    }

    @Override
    public void start() {
        // made here instead of init() so the alliance picked during init sets the tag pipeline
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(startX, startY, Math.toRadians(startHeadingDeg)));

        limelight = new Limelight(hardwareMap, alliance);
        limelight.switchToShootPipeline();

        filter.reset();
        loopTimer.reset();
        frameTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.pose();

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, pose.heading()
        );
        follower.manual(powers);

        handleButtons();

        // one read of the limelight per loop
        limelight.periodic();

        // the filter predicts every loop off pedro and corrects only when a new tag frame shows up
        Pose goal = Shooter.closestGoal(pose, alliance);
        double fresh = limelight.freshDistanceToGoal();
        odometryDistance = pose.distance(goal);
        filtered = filter.update(pose, goal, fresh);

        if (!Double.isNaN(fresh)) {
            lastLimelight = fresh;
            addSample(fresh, filtered, odometryDistance);
        }

        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Closest goal", "(%.1f, %.1f)", goal.x(), goal.y());
        telemetry.addData("Loop", "%.0f Hz   limelight %.1f Hz", 1000 / loopMs, frameHz);
        telemetry.addLine();

        addFilterTelemetry();
        telemetry.addLine();
        addStatsTelemetry();
        telemetry.addLine();
        addTagTelemetry();

        telemetry.update();
    }

    private void handleButtons() {
        if (gamepad1.dpadUpWasPressed()) setWeight(GoalDistanceFilter.limelightWeight + weightStep);
        if (gamepad1.dpadDownWasPressed()) setWeight(GoalDistanceFilter.limelightWeight - weightStep);

        if (gamepad1.dpadRightWasPressed()) Limelight.distanceOffset += offsetStep;
        if (gamepad1.dpadLeftWasPressed()) Limelight.distanceOffset -= offsetStep;

        if (gamepad1.aWasPressed()) resetFilter();
        if (gamepad1.bWasPressed()) resetStats();

        if (gamepad1.xWasPressed()) {
            // odometry has drifted, put it back where we know the robot is
            follower.setPose(new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
            resetFilter(); // the pose jumped, the filter would read it as motion
        }

        if (gamepad1.yWasPressed()) setPedroOnly(!pedroOnly);

        if (gamepad1.backWasPressed()) {
            resetFilter();
            resetStats();
        }
    }

    private void setWeight(double weight) {
        pedroOnly = false;
        GoalDistanceFilter.limelightWeight = Math.max(0, Math.min(1, weight));
    }

    // holds the filter on pedro alone so you can watch the odometry walk away from the limelight
    private void setPedroOnly(boolean on) {
        if (on == pedroOnly) return;

        if (on) {
            savedWeight = GoalDistanceFilter.limelightWeight;
            GoalDistanceFilter.limelightWeight = 0;
        } else {
            GoalDistanceFilter.limelightWeight = savedWeight;
        }
        pedroOnly = on;
    }

    private void resetFilter() {
        filter.reset();
        lastLimelight = Double.NaN;
        filtered = 0;
    }

    private void resetStats() {
        samples.clear();
        frames = 0;
        frameTimer.reset();
        frameHz = 0;
    }

    private void addSample(double raw, double filteredValue, double odometry) {
        samples.addLast(new double[] {raw, filteredValue, odometry});
        while (samples.size() > Math.max(2, statSamples))
            samples.removeFirst();

        frames++;
        if (frameTimer.seconds() > 1) {
            frameHz = frames / frameTimer.seconds();
            frames = 0;
            frameTimer.reset();
        }
    }

    private void addFilterTelemetry() {
        telemetry.addLine("--- Filter (a = reset, y = pedro only) ---");
        telemetry.addData("Mode", pedroOnly ? "PEDRO ONLY (y to go back)" : "blended");
        telemetry.addData("limelightWeight", "%.2f  (dpad up/down)", GoalDistanceFilter.limelightWeight);
        telemetry.addData("distanceOffset", "%.1f in  (dpad left/right)", Limelight.distanceOffset);
        telemetry.addData("maxJump / maxRejects", "%.1f in / %d  (dashboard)",
                GoalDistanceFilter.maxJump, GoalDistanceFilter.maxRejects);
        telemetry.addLine();

        telemetry.addData("Odometry distance", "%.2f in", odometryDistance);
        telemetry.addData("Limelight distance", fmt(limelight.distanceToGoal()));
        telemetry.addData("FILTERED distance", "%.2f in  <- what the shooter uses", filtered);
        telemetry.addData("Filtered - limelight", fmt(Double.isNaN(lastLimelight) ? Double.NaN : filtered - lastLimelight));
        telemetry.addData("Filtered - odometry", "%.2f in", filtered - odometryDistance);
        telemetry.addLine();

        telemetry.addData("dx from pedro", "%.3f in/loop", filter.getDx());
        telemetry.addData("Corrections", filter.getCorrections());
        telemetry.addData("REJECTS in a row", filter.getRejects() + (filter.getRejects() > 0 ? "  <- maxJump too tight?" : ""));
    }

    private void addStatsTelemetry() {
        telemetry.addLine("--- Noise, last " + samples.size() + " frames (b = reset, STAND STILL) ---");
        if (samples.size() < 2) {
            telemetry.addLine("not enough tag frames yet");
            return;
        }

        double rawSd = stdDev(0), filteredSd = stdDev(1);
        telemetry.addData("Raw limelight", "mean %.2f  sd %.2f  range %.2f in", mean(0), rawSd, range(0));
        telemetry.addData("Filtered", "mean %.2f  sd %.2f  range %.2f in", mean(1), filteredSd, range(1));
        telemetry.addData("Noise left after filtering", rawSd > 0 ? String.format(Locale.US, "%.0f%%", 100 * filteredSd / rawSd) : "-");

        if (trueDistance > 0) {
            telemetry.addLine();
            telemetry.addData("True distance", "%.2f in", trueDistance);
            telemetry.addData("Limelight error", "%.2f in  <- fix with distanceOffset", mean(0) - trueDistance);
            telemetry.addData("Filtered error", "%.2f in", mean(1) - trueDistance);
            telemetry.addData("Odometry error", "%.2f in  <- pedro drift", mean(2) - trueDistance);
        } else {
            telemetry.addLine("set trueDistance on the dashboard to score the errors");
        }
    }

    private void addTagTelemetry() {
        telemetry.addLine("--- Tags (all of one cell should match) ---");

        Map<Integer, Double> distances = limelight.tagDistances();
        if (distances.isEmpty()) {
            telemetry.addLine("no tag in view");
            return;
        }

        double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE;
        for (Map.Entry<Integer, Double> tag : distances.entrySet()) {
            double d = tag.getValue();
            lo = Math.min(lo, d);
            hi = Math.max(hi, d);
            telemetry.addData("Tag " + tag.getKey(), "%.2f in   offset %.2f in", d, Limelight.goalOffset(tag.getKey()));
        }

        if (distances.size() > 1)
            telemetry.addData("SPREAD", "%.2f in  <- drive this to 0 with the offsets", hi - lo);
        telemetry.addData("Live offset", Limelight.tuneTagId < 0
                ? "off (set Limelight.tuneTagId to a tag id)"
                : String.format(Locale.US, "tag %d = %.2f in", Limelight.tuneTagId, Limelight.tuneOffset));
        telemetry.addData("Angle to goal center", fmtDeg(limelight.angle()));
    }

    private double mean(int column) {
        double sum = 0;
        for (double[] s : samples) sum += s[column];
        return sum / samples.size();
    }

    private double stdDev(int column) {
        double m = mean(column), sum = 0;
        for (double[] s : samples) sum += (s[column] - m) * (s[column] - m);
        return Math.sqrt(sum / (samples.size() - 1));
    }

    private double range(int column) {
        double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE;
        for (double[] s : samples) {
            lo = Math.min(lo, s[column]);
            hi = Math.max(hi, s[column]);
        }
        return hi - lo;
    }

    private static String fmt(double inches) {
        return Double.isNaN(inches) ? "no tag" : String.format(Locale.US, "%.2f in", inches);
    }

    private static String fmtDeg(double degrees) {
        return Double.isNaN(degrees) ? "no tag" : String.format(Locale.US, "%.2f deg", degrees);
    }

    @Override
    public void stop() {
        // null if stopped during init
        if (limelight != null) limelight.stop();
        setPedroOnly(false); // don't leave the dashboard weight on 0 for the next opmode
    }
}
