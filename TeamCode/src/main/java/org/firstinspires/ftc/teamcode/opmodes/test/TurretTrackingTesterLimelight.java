package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.Robot;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.commandbase.util.GoalAngleFilter;

import java.util.Arrays;
import java.util.Locale;

// aims the turret off the LIMELIGHT ONLY (Turret.AimSource.LIMELIGHT): every new tag frame sets
// target = turret + cameraYawOffset - tx, and between frames (or with no tag) it holds the last target.
// the cell comes from whichever of our alliance's cells has a tag in view (Robot.visibleGoal), so the
// pedro pose doesn't need to be right. other alliance tags never count: check Accepted tags
// pedro isn't used for aiming or feedforward, it's only read for the AIM ERROR (pedro) line
// sticks = field-centric drive   start = reset heading   dpad up = set pose to startX/Y/Heading
// a = tracking on/off   b = center turret   x = reset stats
// init: back = resync turret (point the shooter forward first)
//
// AIM ERROR (LL) = tx - cameraYawOffset, how far the shooter is off the goal according to the limelight
// graph LL Goal Angle, Target Angle and Turret Angle in Panels/Dashboard
//
// HOW TO TEST (run TurretTester first so the encoder and limits are set up):
// 1. Set blue to your alliance BEFORE init. Point the robot roughly at the goal so the tags are in view.
// 2. Press a. AIM ERROR (LL) should settle near 0 and stay there while sitting still.
//    If it settles off by a constant, that's cameraYawOffset. If it oscillates, the PID is chasing
//    the ~20Hz frame steps: lower kP or check LL frames Hz.
// 3. Turn slowly. It should follow, but lag behind (no kV feedforward here) and catch up each frame.
//    This lag is what pedro fixes in the fused tracking tester.
// 4. Turn faster. Once Turret speed passes the LL cutoff the frames are ignored and the turret freezes,
//    so it can fall off the goal. That's expected, raise Turret.maxCorrectionSpeed only if the frames look clean.
// 5. Cover the limelight. The turret should hold still at its last target (not drift or snap).
// no spin test here: the goal goes behind the robot with no tag in view, so it can't wrap
@Configurable
@Config
@TeleOp
public class TurretTrackingTesterLimelight extends OpMode {
    public static boolean blue = false;
    public static double startX = 72, startY = 72, startHeading = 0; // in, in, deg

    private Robot robot;
    private JoinedTelemetry joinedTelemetry;

    private double maxAimError = 0;
    private int lastTagFrames = 0;
    private long lastFrameNanos = 0;
    private double frameHz = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, blue ? Alliance.BLUE : Alliance.RED);
        robot.turret.setAimSource(Turret.AimSource.LIMELIGHT);
        robot.setPose(startPose());
        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
        );
    }

    @Override
    public void init_loop() {
        if (gamepad1.backWasPressed()) robot.turret.resync();

        robot.periodic();
        joinedTelemetry.addData("Aim source", robot.turret.getAimSource());
        joinedTelemetry.addData("Alliance", robot.alliance);
        joinedTelemetry.addData("Turret Angle", robot.turret.getAngle());
        joinedTelemetry.addLine("back to resync if the shooter faces forward and Turret Angle isn't ~0");
        joinedTelemetry.addLine("a after start to begin tracking");
        joinedTelemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            if (robot.turret.isTracking()) robot.turret.stop();
            else robot.turret.startTracking();
            maxAimError = 0;
        }
        if (gamepad1.bWasPressed()) robot.turret.setTarget(0);
        if (gamepad1.xWasPressed()) maxAimError = 0;
        if (gamepad1.startWasPressed()) robot.resetHeading();
        if (gamepad1.dpadUpWasPressed()) robot.setPose(startPose());

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x,
                robot.follower.pose().heading()
        );
        robot.follower.manual(powers);

        robot.periodic();
        updateStats();
        updateTelemetry();
    }

    private Pose startPose() {
        return new Pose(startX, startY, Math.toRadians(startHeading));
    }

    private void updateStats() {
        int frames = robot.limelight.getTagFrames();
        if (frames != lastTagFrames) {
            long now = System.nanoTime();
            if (lastFrameNanos != 0) frameHz = (frames - lastTagFrames) * 1e9 / (now - lastFrameNanos);
            lastFrameNanos = now;
            lastTagFrames = frames;
        }

        Turret turret = robot.turret;
        if (!turret.isTracking() || turret.isClamped() || Math.abs(turret.getError()) >= Turret.wrapError) return;
        double error = aimError();
        if (!Double.isNaN(error)) maxAimError = Math.max(maxAimError, Math.abs(error));
    }

    // deg the turret is off from where the limelight says the goal is, NaN with no tag on our cell in view.
    // the limelight's goal angle is turret + cameraYawOffset - tx, so turret - that = tx - cameraYawOffset.
    // uses the same cell's tags the turret aims with, not just the closest tag of any cell
    private double aimError() {
        double tx = robot.limelight.angle(robot.closestCellTags());
        return Double.isNaN(tx) ? Double.NaN : GoalAngleFilter.wrap(tx - Turret.cameraYawOffset);
    }

    // deg the turret is off from where pedro says the goal is, just for comparison
    private double pedroAimError() {
        return GoalAngleFilter.wrap(robot.turret.getAngle() - robot.turret.getOdometryAngle());
    }

    private void updateTelemetry() {
        Turret turret = robot.turret;
        Pose pose = robot.follower.pose();
        JoinedTelemetry t = joinedTelemetry;

        t.addData("Aim source", turret.getAimSource());
        t.addData("LL Goal Angle", turret.getLimelightAngle());
        t.addData("Target Angle", turret.getTargetDegrees());
        t.addData("Turret Angle", turret.getAngle());
        t.addData("Error (deg)", Turret.ticksToDegrees(turret.getError()));
        t.addData("Power", turret.getPower());
        t.addLine();

        t.addData("Mode", !turret.isEnabled() ? "OFF" : turret.isTracking() ? "TRACKING" : "HOLDING");
        double error = aimError();
        t.addData("AIM ERROR (LL)", Double.isNaN(error) ? "no tag" : String.format(Locale.US, "%.2f deg  (max %.2f)", error, maxAimError));
        t.addData("At target", turret.atTarget());
        t.addData("Clamped", turret.isClamped());
        t.addLine();

        int[] cell = robot.closestCellTags();
        t.addData("LL tags (all)", robot.limelight.visibleTagIds().toString());
        t.addData("Cell tags", Arrays.toString(cell));
        t.addData("Accepted tags", robot.acceptedTagIds().toString());
        int usedTag = robot.limelight.closestTagId(cell);
        t.addData("Aiming off tag", usedTag < 0 ? "none" : String.valueOf(usedTag));
        t.addData("LL raw tx", String.format(Locale.US, "%.2f deg  (limelight's own, no offset/flip)", robot.limelight.rawTx(cell)));
        t.addData("LL goal tx", String.format(Locale.US, "%.2f deg  (to cell center, flipped. camera yaw %.2f)",
                robot.limelight.angle(cell), Turret.cameraYawOffset));
        t.addData("LL frames", String.format(Locale.US, "%d  (%.0f Hz)", robot.limelight.getTagFrames(), frameHz));
        t.addData("Turret speed", String.format(Locale.US, "%.0f deg/s  (LL ignored above %.0f)",
                Turret.ticksToDegrees(Math.abs(turret.getVelocity())), Turret.ticksToDegrees(Turret.maxCorrectionSpeed)));
        t.addLine();

        t.addData("Pose", String.format(Locale.US, "x %.1f  y %.1f  h %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        Pose goal = robot.getGoal(); // the cell the limelight sees, or pedro's pick with no tag in view
        t.addData("Goal", String.format(Locale.US, "x %.0f  y %.0f  %.1f in away (pedro)",
                goal.x(), goal.y(), pose.distance(goal)));
        t.addData("Aim error (pedro, not used)", String.format(Locale.US, "%.2f deg", pedroAimError()));
        t.addData("Loop", String.format(Locale.US, "%.0f Hz", robot.getLoopTimeHz()));
        t.update();
    }

    @Override
    public void stop() {
        robot.turret.stop();
    }
}
