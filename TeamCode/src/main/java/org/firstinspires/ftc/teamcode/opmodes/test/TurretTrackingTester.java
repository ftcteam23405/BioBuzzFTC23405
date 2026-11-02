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

// Runs the real tracking loop (Robot.periodic -> Turret.aim -> filter -> unwrap -> PID) while you drive, so you
// can check the turret stays on the goal. Graph Odometry Angle, Target Angle and Turret Angle in Panels/Dashboard.
// sticks = field-centric drive   start = reset heading   dpad up = set pose to startX/Y/Heading
// a = tracking on/off   b = center turret   y = spin in place at spinPower (watch it wrap)   x = reset stats
// init: back = resync turret (point the shooter forward first)
//
// HOW TO TEST (run TurretTester first so the encoder and limits are set up):
// 1. Set blue to your alliance BEFORE init (it picks the limelight pipeline). Put the robot at startX/Y/Heading.
// 2. Press a. The turret should swing to the goal, AIM ERROR (pedro) should settle near 0.
// 3. Drive and turn slowly. AIM ERROR should stay within a few deg. If it lags while turning, raise Turret.kV.
// 4. Cover the limelight. LL Corrections should stop climbing, but it should keep tracking on pedro alone.
// 5. Press y. Each time the goal goes behind the robot the turret should stop at the limit (Clamped), wait
//    until the goal is wrapBuffer past it, then swing a full turn ONCE. Wraps should go up by 1 per turn.
//    If Wraps climbs fast while the goal sits behind the robot, raise Turret.wrapBuffer.
// 6. LL tx should read about Turret.cameraYawOffset when the shooter is centered on the goal.
@Configurable
@Config
@TeleOp
public class TurretTrackingTester extends OpMode {
    public static boolean blue = false;
    public static double startX = 72, startY = 72, startHeading = 0; // in, in, deg
    public static double spinPower = 0.3; // turn power for y, negative spins the other way

    private Robot robot;
    private JoinedTelemetry joinedTelemetry;

    private boolean spinning = false;
    private int wraps = 0;
    private double lastTarget = Double.NaN;
    private double maxAimError = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, blue ? Alliance.BLUE : Alliance.RED);
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
                robot.follower.pose().heading(), robot.alliance == Alliance.BLUE ? Math.toRadians(180) : 0
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
        if (!turret.isClamped() && Math.abs(turret.getError()) < Turret.wrapError)
            maxAimError = Math.max(maxAimError, Math.abs(aimError()));
    }

    private void resetStats() {
        wraps = 0;
        maxAimError = 0;
        lastTarget = Double.NaN;
    }

    // deg the turret is off from where pedro says the goal is
    private double aimError() {
        return GoalAngleFilter.wrap(robot.turret.getAngle() - robot.turret.getOdometryAngle());
    }

    private void updateTelemetry() {
        Turret turret = robot.turret;
        Pose pose = robot.follower.pose();

        joinedTelemetry.addData("Odometry Angle", turret.getOdometryAngle());
        joinedTelemetry.addData("Target Angle", turret.getTargetDegrees());
        joinedTelemetry.addData("Turret Angle", turret.getAngle());
        joinedTelemetry.addData("Error (deg)", Turret.ticksToDegrees(turret.getError()));
        joinedTelemetry.addData("Power", turret.getPower());
        joinedTelemetry.addLine();

        joinedTelemetry.addData("Mode", !turret.isEnabled() ? "OFF" : turret.isTracking() ? "TRACKING" : "HOLDING");
        joinedTelemetry.addData("Spinning", spinning);
        joinedTelemetry.addData("Pose", String.format(Locale.US, "x %.1f  y %.1f  h %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        Pose goal = Shooter.closestGoal(pose, robot.alliance); // same pick as Robot, minus its stickiness
        joinedTelemetry.addData("Goal", String.format(Locale.US, "x %.0f  y %.0f  %.1f in away",
                goal.x(), goal.y(), pose.distance(goal)));
        joinedTelemetry.addLine();

        joinedTelemetry.addData("AIM ERROR (pedro)", String.format(Locale.US, "%.2f deg  (max %.2f)", aimError(), maxAimError));
        joinedTelemetry.addData("At target", turret.atTarget());
        joinedTelemetry.addData("Clamped", turret.isClamped());
        joinedTelemetry.addData("Wraps", wraps);
        joinedTelemetry.addData("Target rate", String.format(Locale.US, "%.1f deg/s", Turret.ticksToDegrees(turret.getTargetRate())));
        joinedTelemetry.addLine();

        joinedTelemetry.addData("LL goal angle", String.format(Locale.US, "%.2f deg", turret.getLimelightAngle()));
        joinedTelemetry.addData("LL tx", String.format(Locale.US, "%.2f deg  (camera yaw %.2f)", robot.limelight.angle(), Turret.cameraYawOffset));
        joinedTelemetry.addData("LL tags", robot.limelight.visibleTagIds().toString());
        joinedTelemetry.addData("LL corrections", turret.getFilterCorrections());
        joinedTelemetry.addData("LL rejects", turret.getFilterRejects());
        joinedTelemetry.addData("Filter dx", String.format(Locale.US, "%.2f deg", turret.getFilterDx()));
        joinedTelemetry.addLine();

        joinedTelemetry.addData("Limits", String.format(Locale.US, "+/-%.0f deg  wrap buffer %.1f deg",
                Turret.ticksToDegrees(Turret.maxTicks), Turret.ticksToDegrees(Turret.wrapBuffer)));
        joinedTelemetry.addData("Loop", String.format(Locale.US, "%.0f Hz", robot.getLoopTimeHz()));
        joinedTelemetry.update();
    }

    @Override
    public void stop() {
        robot.turret.stop();
    }
}
