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
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.commandbase.util.GoalAngleFilter;

import java.util.Locale;

// aims the turret off PINPOINT (pedro's pose) ONLY (Turret.AimSource.ODOMETRY): target = angle from the robot
// pose to the goal, with kV feedforward from how fast that angle is changing. the limelight is still read, but
// only for the AIM ERROR (LL) line, which shows how far off pedro's pose really is
// sticks = field-centric drive   start = reset heading   dpad up = set pose to startX/Y/Heading
// a = tracking on/off   b = center turret   y = spin in place at spinPower (watch it wrap)   x = reset stats
// init: back = resync turret (point the shooter forward first)
//
// graph Odometry Angle, Target Angle and Turret Angle in Panels/Dashboard
//
// HOW TO TEST (run TurretTester first so the encoder and limits are set up):
// 1. Set blue to your alliance BEFORE init. Put the robot EXACTLY at startX/Y/Heading, any pose error
//    shows up straight in the aim since nothing corrects it.
// 2. Press a. AIM ERROR (pedro) is the control error: it should settle near 0.
// 3. Drive and turn. AIM ERROR (pedro) should stay within a few deg. If it lags while turning, raise
//    Turret.kV. If it overshoots after a turn stops, lower it. This tester is the cleanest place to tune kV.
// 4. Watch AIM ERROR (LL) whenever a tag is in view. Right after a dpad up reset it should be near 0
//    (if not, check startX/Y/Heading, cameraYawOffset or pivotForward/pivotLeft). As you drive around it
//    grows as pedro drifts, that drift is what the limelight corrects in the fused tracking tester.
// 5. Cover the limelight. Nothing should change, the limelight isn't aiming.
// 6. Press y. Each time the goal goes behind the robot the turret should stop at the limit (Clamped), wait
//    until the goal is wrapBuffer past it, then swing a full turn ONCE. Wraps should go up by 1 per turn.
//    If Wraps climbs fast while the goal sits behind the robot, raise Turret.wrapBuffer.
@Configurable
@Config
@TeleOp
public class TurretTrackingTesterPinpoint extends OpMode {
    public static boolean blue = false;
    public static double startX = 72, startY = 72, startHeading = 0; // in, in, deg
    public static double spinPower = 0.3; // turn power for y, negative spins the other way

    private Robot robot;
    private JoinedTelemetry joinedTelemetry;

    private boolean spinning = false;
    private int wraps = 0;
    private double lastTarget = Double.NaN;
    private double maxAimError = 0, maxLimelightError = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, blue ? Alliance.BLUE : Alliance.RED);
        robot.turret.setAimSource(Turret.AimSource.ODOMETRY);
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
            resetStats();
        }
        if (gamepad1.bWasPressed()) robot.turret.setTarget(0);
        if (gamepad1.yWasPressed()) spinning = !spinning;
        if (gamepad1.xWasPressed()) resetStats();
        if (gamepad1.startWasPressed()) robot.resetHeading();
        if (gamepad1.dpadUpWasPressed()) robot.setPose(startPose());

        double turn = spinning ? spinPower : -gamepad1.right_stick_x;
        DrivePowers powers = ManualDrive.fieldCentric(
                spinning ? 0 : -gamepad1.left_stick_y, spinning ? 0 : -gamepad1.left_stick_x, turn,
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

    // a target jump of more than half a turn in one loop is unwrap() switching sides
    private void updateStats() {
        Turret turret = robot.turret;
        if (!turret.isTracking()) {
            lastTarget = Double.NaN;
            return;
        }

        double target = turret.getTarget();
        if (!Double.isNaN(lastTarget) && Math.abs(target - lastTarget) > Turret.ticksPerTurretRev() / 2) wraps++;
        lastTarget = target;

        // only count it once the turret has had a chance to get there, not mid-wrap or pinned at a limit
        if (!turret.isClamped() && Math.abs(turret.getError()) < Turret.wrapError) {
            maxAimError = Math.max(maxAimError, Math.abs(aimError()));
            double ll = limelightAimError();
            if (!Double.isNaN(ll)) maxLimelightError = Math.max(maxLimelightError, Math.abs(ll));
        }
    }

    private void resetStats() {
        wraps = 0;
        maxAimError = 0;
        maxLimelightError = 0;
        lastTarget = Double.NaN;
    }

    // deg the turret is off from where pedro says the goal is
    private double aimError() {
        return GoalAngleFilter.wrap(robot.turret.getAngle() - robot.turret.getOdometryAngle());
    }

    // deg the turret is off from where the limelight says the goal is, NaN with no tag on our cell in view.
    // the limelight's goal angle is turret + cameraYawOffset - tx, so turret - that = tx - cameraYawOffset
    private double limelightAimError() {
        if (!robot.closestCellTagsVisible()) return Double.NaN;
        double tx = robot.limelight.angle();
        return Double.isNaN(tx) ? Double.NaN : GoalAngleFilter.wrap(tx - Turret.cameraYawOffset);
    }

    private void updateTelemetry() {
        Turret turret = robot.turret;
        Pose pose = robot.follower.pose();
        JoinedTelemetry t = joinedTelemetry;

        t.addData("Aim source", turret.getAimSource());
        t.addData("Odometry Angle", turret.getOdometryAngle());
        t.addData("Target Angle", turret.getTargetDegrees());
        t.addData("Turret Angle", turret.getAngle());
        t.addData("Error (deg)", Turret.ticksToDegrees(turret.getError()));
        t.addData("Power", turret.getPower());
        t.addLine();

        t.addData("Mode", !turret.isEnabled() ? "OFF" : turret.isTracking() ? "TRACKING" : "HOLDING");
        t.addData("Spinning", spinning);
        t.addData("Pose", String.format(Locale.US, "x %.1f  y %.1f  h %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        Pose goal = Shooter.closestGoal(pose); // same pick as Robot, minus its stickiness
        t.addData("Goal", String.format(Locale.US, "x %.0f  y %.0f  %.1f in away",
                goal.x(), goal.y(), pose.distance(goal)));
        t.addLine();

        t.addData("AIM ERROR (pedro)", String.format(Locale.US, "%.2f deg  (max %.2f)", aimError(), maxAimError));
        double ll = limelightAimError();
        t.addData("AIM ERROR (LL)", Double.isNaN(ll) ? "no tag" : String.format(Locale.US, "%.2f deg  (max %.2f)", ll, maxLimelightError));
        t.addData("At target", turret.atTarget());
        t.addData("Clamped", turret.isClamped());
        t.addData("Wraps", wraps);
        t.addData("Target rate", String.format(Locale.US, "%.1f deg/s", Turret.ticksToDegrees(turret.getTargetRate())));
        t.addLine();

        t.addData("LL tx (not used)", String.format(Locale.US, "%.2f deg  (camera yaw %.2f)", robot.limelight.angle(), Turret.cameraYawOffset));
        t.addData("LL tags", robot.limelight.visibleTagIds().toString());
        t.addData("Limits", String.format(Locale.US, "%.0f to %.0f deg  wrap buffer %.1f deg",
                Turret.ticksToDegrees(Turret.minTicks), Turret.ticksToDegrees(Turret.maxTicks),
                Turret.ticksToDegrees(Turret.wrapBuffer)));
        t.addData("Loop", String.format(Locale.US, "%.0f Hz", robot.getLoopTimeHz()));
        t.update();
    }

    @Override
    public void stop() {
        robot.turret.stop();
    }
}
