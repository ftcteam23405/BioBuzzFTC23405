package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

// builds the distance -> velocity table in Shooter, using only the Limelight and Shooter subsystems
// during init: dpad left = blue, dpad right = red
// dpad up/down = velocity +/- step, a = spin up to that velocity, b = spin down
// x = record (distance, velocity), back = clear recorded points
// y = toggle table mode: velocity follows the limelight distance through Shooter's table, to check it
// how to use: park at a distance, adjust velocity until shots go in, press x, repeat at a few distances,
// then copy the DISTANCES / VELOCITIES lines from telemetry into Shooter
@Configurable
@Config
@TeleOp
public class ShooterTuner extends OpMode {
    public static double step = 25;          // ticks/s per dpad press
    public static double startVelocity = 1300;
    public static double ticksPerRev = 28;   // flywheel motor encoder, for the rpm readout
    public static int averageSamples = 15;   // limelight readings averaged for the recorded distance

    private Limelight limelight;
    private Shooter shooter;
    private Alliance alliance = Alliance.BLUE;

    private double manualVelocity = startVelocity;
    private boolean tableMode = false;

    private final ArrayDeque<Double> recentDistances = new ArrayDeque<>();
    private final List<double[]> points = new ArrayList<>(); // {distance, velocity}

    @Override
    public void init() {
        manualVelocity = startVelocity;
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadLeftWasPressed()) alliance = Alliance.BLUE;
        if (gamepad1.dpadRightWasPressed()) alliance = Alliance.RED;

        telemetry.addData("Alliance", alliance + " (dpad left/right to change)");
        telemetry.update();
    }

    @Override
    public void start() {
        // made here instead of init() so the alliance picked during init sets the tag pipeline
        limelight = new Limelight(hardwareMap, alliance);
        limelight.switchToShootPipeline();
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void loop() {
        limelight.periodic(); // one read of the limelight per loop

        double distance = limelight.distanceToGoal();            // last reading, NaN if no tag
        double freshDistance = limelight.freshDistanceToGoal();  // NaN unless it's a new frame

        // only average new frames, otherwise the same reading fills the window and the average is a lie
        updateAverage(freshDistance);

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
        if (gamepad1.xWasPressed()) record();
        if (gamepad1.backWasPressed()) points.clear();

        // only update on a new tag frame, otherwise hold the last target
        if (tableMode)
            shooter.setVelocityFromDistance(freshDistance);

        shooter.periodic();

        telemetry.addData("Mode", tableMode ? "TABLE (y to go back to manual)" : "MANUAL (y for table mode)");
        telemetry.addData("Alliance", alliance);
        telemetry.addLine();

        //limelight telemetry
        telemetry.addData("Distance to goal", Double.isNaN(distance) ? "no tag" : String.format(Locale.US, "%.1f in", distance));
        telemetry.addData("Averaged distance", recentDistances.isEmpty() ? "no tag" : String.format(Locale.US, "%.1f in (%d samples)", averageDistance(), recentDistances.size()));
        telemetry.addData("Visible tags", limelight.visibleTagIds());
        telemetry.addData("Tag frames", limelight.getTagFrames());
        telemetry.addData("Angle to goal center (deg)", "%.2f", limelight.angle());
        telemetry.addLine();

        //shooter telemetry
        telemetry.addData("Manual velocity", "%.0f ticks/s (%.0f rpm)", manualVelocity, toRpm(manualVelocity));
        telemetry.addData("Target velocity", "%.0f ticks/s (%.0f rpm)", shooter.getTargetVelocity(), toRpm(shooter.getTargetVelocity()));
        telemetry.addData("Actual velocity", "%.0f ticks/s (%.0f rpm)", shooter.getVelocity(), toRpm(shooter.getVelocity()));
        telemetry.addData("At target", shooter.atTarget());
        if (!recentDistances.isEmpty())
            telemetry.addData("Current table says", "%.0f ticks/s", Shooter.velocityFor(averageDistance()));
        if (tableMode)
            telemetry.addData("Filtered distance", "%.1f in", shooter.getGoalDistance());
        telemetry.addLine();

        addRecordedTelemetry();
        telemetry.update();
    }

    private void setManualVelocity(double velocity) {
        manualVelocity = Math.max(0, velocity);
        tableMode = false;
        shooter.setTargetVelocity(manualVelocity);
    }

    // average of the last few tag readings, so one noisy frame doesn't end up in the table
    private void updateAverage(double distance) {
        if (Double.isNaN(distance)) return;

        recentDistances.addLast(distance);
        while (recentDistances.size() > Math.max(1, averageSamples))
            recentDistances.removeFirst();
    }

    private double averageDistance() {
        double sum = 0;
        for (double d : recentDistances) sum += d;
        return sum / recentDistances.size();
    }

    private void record() {
        if (recentDistances.isEmpty()) return; // no tag seen yet, nothing to record

        points.add(new double[] {averageDistance(), shooter.getTargetVelocity()});
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
        if (limelight != null) limelight.stop();
    }
}
