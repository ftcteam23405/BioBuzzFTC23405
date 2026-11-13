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

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

// builds the distance -> velocity table in Shooter, using the pinpoint (pedro follower) distance to the closest goal
// the robot has to start on the start pose below (field center by default), or every distance will be off
// driving: left stick translate, right stick turn (field centric)
// dpad up/down = velocity +/- step, a = spin up to that velocity, b = spin down
// x = record (distance, velocity), back = clear recorded points, left bumper = put the follower back on the start pose
// y = toggle table mode: velocity follows the pinpoint distance through Shooter's table, to check it
// right trigger = intake
// how to use: drive to a distance, adjust velocity until shots go in, press x, repeat at a few distances,
// then copy the DISTANCES / VELOCITIES lines from telemetry into Shooter
@Configurable
@Config
@TeleOp
public class ShooterTuner extends OpMode {
    public static double step = 25;          // ticks/s per dpad press
    public static double startVelocity = 1300;
    public static double ticksPerRev = 28;   // flywheel motor encoder, for the rpm readout
    public static double startX = 72, startY = 72, startHeadingDeg = 0; // field center

    private Follower follower;;
    private Shooter shooter;
    private Intake intake;

    private Pose goal = null; // current closest goal, kept so it doesn't flicker between the two cells
    private double manualVelocity = startVelocity;
    private boolean tableMode = false;

    private final List<double[]> points = new ArrayList<>(); // {distance, velocity}

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        manualVelocity = startVelocity;
    }

    @Override
    public void init_loop() {
        telemetry.addData("Start pose", "(%.1f, %.1f) %.0f deg", startX, startY, startHeadingDeg);
        telemetry.addLine("put the robot on the start pose, the distance comes from odometry");
        telemetry.update();
    }

    @Override
    public void start() {
        follower = Constants.create(hardwareMap);
        resetPose();
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.pose();

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, pose.heading()
        );
        follower.manual(powers);

        goal = Shooter.closestGoal(pose, goal);
        double distance = pose.distance(goal);

        if (gamepad1.dpadUpWasPressed()) setManualVelocity(manualVelocity + step);
        if (gamepad1.dpadDownWasPressed()) setManualVelocity(manualVelocity - step);
        if (gamepad1.aWasPressed()) setManualVelocity(manualVelocity);
        if (gamepad1.bWasPressed()) {
            tableMode = false;
            shooter.stop();
        }
        if (gamepad1.yWasPressed()) {
            tableMode = !tableMode;
            if (tableMode) shooter.startTracking();
            else setManualVelocity(manualVelocity);
        }
        if (gamepad1.xWasPressed()) record(distance);
        if (gamepad1.backWasPressed()) points.clear();
        if (gamepad1.leftBumperWasPressed()) resetPose();

        if (gamepad1.rightTriggerWasPressed()) {
            intake.intakeOn();
        }
        if (gamepad1.rightTriggerWasReleased()) {
            intake.intakeOff();
        }

        if (tableMode)
            shooter.setVelocityFromDistance(pose, goal);

        shooter.periodic();

        telemetry.addData("Mode", tableMode ? "TABLE (y to go back to manual)" : "MANUAL (y for table mode)");
        telemetry.addLine();

        //pinpoint telemetry
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Closest goal", "(%.0f, %.0f)", goal.x(), goal.y());
        telemetry.addData("Distance to goal", "%.1f in", distance);
        telemetry.addLine();

        //shooter telemetry
        telemetry.addData("Manual velocity", "%.0f ticks/s (%.0f rpm)", manualVelocity, toRpm(manualVelocity));
        telemetry.addData("Target velocity", "%.0f ticks/s (%.0f rpm)", shooter.getTargetVelocity(), toRpm(shooter.getTargetVelocity()));
        telemetry.addData("Actual velocity", "%.0f ticks/s (%.0f rpm)", shooter.getVelocity(), toRpm(shooter.getVelocity()));
        telemetry.addData("At target", shooter.atTarget());
        telemetry.addData("Current table says", "%.0f ticks/s", Shooter.velocityFor(distance));
        telemetry.addLine();

        addRecordedTelemetry();
        telemetry.update();
    }

    private void resetPose() {
        follower.setPose(new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
        goal = null;
    }

    private void setManualVelocity(double velocity) {
        manualVelocity = Math.max(0, velocity);
        tableMode = false;
        shooter.setTargetVelocity(manualVelocity);
    }

    private void record(double distance) {
        points.add(new double[] {distance, shooter.getTargetVelocity()});
        points.sort(Comparator.comparingDouble(p -> p[0])); // the table needs increasing distances
    }

    // printed in the same format as Shooter's table so it can be copied straight in
    private void addRecordedTelemetry() {
        telemetry.addLine("--- Recorded (x = add, back = clear) ---");
        if (points.isEmpty()) {
            telemetry.addLine("none yet");
            return;
        }

        StringBuilder distances = new StringBuilder();
        StringBuilder velocities = new StringBuilder();
        for (int i = 0; i < points.size(); i++) {
            String sep = i == 0 ? "" : ", ";
            distances.append(sep).append(String.format(Locale.US, "%.1f", points.get(i)[0]));
            velocities.append(sep).append(String.format(Locale.US, "%.0f", points.get(i)[1]));
        }
        telemetry.addLine("DISTANCES = {" + distances + "};");
        telemetry.addLine("VELOCITIES = {" + velocities + "};");
        if (points.size() < 2)
            telemetry.addLine("need at least 2 points for the table");
    }

    private static double toRpm(double ticksPerSecond) {
        return ticksPerSecond / ticksPerRev * 60;
    }

    @Override
    public void stop() {
        // null if stopped during init
        if (shooter != null) shooter.stop();
    }
}
