package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

import org.firstinspires.ftc.teamcode.commandbase.util.GoalDistanceFilter;

import smile.interpolation.Interpolation;
import smile.interpolation.LinearInterpolation;

@Configurable
@Config
public class Shooter {


    // tune with ShooterAutoTuner
    public static double kS = 0.0957;      // power to overcome friction
    public static double kV = 0.0004183;   // linear term for ff
    public static double kP = 0.0025;
    public static double tolerance = 20; // ticks/s
    public static double nominalVoltage = 12; // V the gains are in, output gets scaled by nominalVoltage / battery



    //nothing should ever be mirrored
    public static double goalX = 58;
    public static double audienceGoalY = 58;
    public static double farGoalY = 83;
    public static double goalSwitchMargin = 6; // in. the other goal has to be this much closer before we switch to it,


    // placeholder values, distances must be increasing
    private static final double[] DISTANCES = {40, 60, 80, 100, 120};
    private static final double[] VELOCITIES = {1200, 1250, 1300, 1350, 1400};
    private static final Interpolation velocityTable = new LinearInterpolation(DISTANCES, VELOCITIES);

    private final DcMotorEx shooterMotor;

    private Servo hood;

    private double closePos = 0.9;
    private final List<VoltageSensor> voltageSensors;

    // only read voltage every period cuz we want low loop time
    private static final double VOLTAGE_PERIOD = 0.1; // s
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private double batteryVoltage = Double.NaN;

    private double targetVelocity = 0;
    private boolean tracking = false; // true = velocity follows distance to goal
    private double openLoopPower = Double.NaN; // not NaN = fixed power, no PID (for tuning the feedforward)
    private double power = 0;

    // pedro pose + limelight = filtered ground distance
    private final GoalDistanceFilter distanceFilter = new GoalDistanceFilter();
    private double goalDistance = 0;

    // raw readings from the last update, for telemetry purpoises
    private double odometryDistance = 0;
    private double limelightDistance = Double.NaN;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        voltageSensors = hardwareMap.getAll(VoltageSensor.class);

        hood = hardwareMap.get(Servo.class, "llServo");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        stop();
    }

    // spin at a fixed velocity, stops distance tracking
    public void setTargetVelocity(double velocity) {
        tracking = false;
        openLoopPower = Double.NaN;
        targetVelocity = Math.max(0, velocity);
    }

    // fixed power with no feedforward or PID, until stop() or setTargetVelocity(). for ShooterAutoTuner
    public void setOpenLoopPower(double power) {
        tracking = false;
        targetVelocity = 0;
        openLoopPower = Range.clip(power, 0, 1);
    }

    // keeps the drift correction learned from the limelight
    public void startTracking() {
        tracking = true;
        openLoopPower = Double.NaN;
    }

    public void stop() {
        setTargetVelocity(0);
        setPower(0);
    }

    public boolean isTracking() {
        return tracking;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getVelocity() {
        return shooterMotor.getVelocity();
    }

    public boolean atTarget() {
        return (targetVelocity > 0) && Math.abs(targetVelocity - getVelocity()) < tolerance;
    }

    public void periodic() {
        if (!Double.isNaN(openLoopPower)) {
            setPower(openLoopPower);
            return;
        }

        if (targetVelocity == 0) {
            setPower(0);
            return;
        }

        double feedforward = kS + kV * targetVelocity;
        double feedback = kP * (targetVelocity - getVelocity());

        double compensation = nominalVoltage / getBatteryVoltage();

        setPower(Range.clip((feedforward + feedback) * compensation, 0, 1));
    }

    // get lowest hub voltage
    public double getBatteryVoltage() {
        if (Double.isNaN(batteryVoltage) || voltageTimer.seconds() > VOLTAGE_PERIOD) {
            voltageTimer.reset();
            double lowest = Double.NaN;
            for (VoltageSensor sensor : voltageSensors) {
                double voltage = sensor.getVoltage();
                if (voltage > 0 && (Double.isNaN(lowest) || voltage < lowest)) lowest = voltage;
            }
            batteryVoltage = Double.isNaN(lowest) ? nominalVoltage : lowest;
        }
        return batteryVoltage;
    }

    private void setPower(double power) {
        this.power = power;
        shooterMotor.setPower(power);
    }

    public double getPower() {
        return power;
    }


    // do not pass distanceToGoal() here
    public void setVelocityFromDistance(Pose robot, double limelightDistance) {
        setVelocityFromDistance(robot, closestGoal(robot), limelightDistance);
    }

    // limelightDistance must come from that goal's tags, see Limelight.freshDistanceToGoal(int[])
    public void setVelocityFromDistance(Pose robot, Pose goal, double limelightDistance) {
        odometryDistance = robot.distance(goal);
        if (!Double.isNaN(limelightDistance)) this.limelightDistance = limelightDistance;

        // pedro dx moves the estimate, the limelight pulls it back when a tag is visible
        goalDistance = distanceFilter.update(robot, goal, limelightDistance);
        targetVelocity = velocityFor(goalDistance);
    }

    //target vel from ll distance only (no pedro), still filtered
    public void setVelocityFromDistance(double distance) {
        if (Double.isNaN(distance)) return; // keep the last target instead of spinning at garbage

        this.limelightDistance = distance;
        goalDistance = distanceFilter.update(distance);
        targetVelocity = velocityFor(goalDistance);
    }

    //helper for vel lookup, clamped to the ends of the table
    public static double velocityFor(double distance) {
        double d = Range.clip(distance, DISTANCES[0], DISTANCES[DISTANCES.length - 1]);
        return velocityTable.interpolate(d);
    }

    // the one of our 2 cells that's closer to the robot
    public static Pose closestGoal(Pose robot) {
        return closestGoal(robot, null);
    }

    // sticks with current goal, null for none
    public static Pose closestGoal(Pose robot, Pose current) {
        Pose audienceGoal = new Pose(goalX, audienceGoalY);
        Pose farGoal = new Pose(goalX, farGoalY);
        double toAudience = robot.distance(audienceGoal);
        double toFar = robot.distance(farGoal);

        if (current != null && current.x() == goalX) {
            if (current.y() == audienceGoalY && toAudience < toFar + goalSwitchMargin) return audienceGoal;
            if (current.y() == farGoalY && toFar < toAudience + goalSwitchMargin) return farGoal;
        }
        return toAudience < toFar ? audienceGoal : farGoal;
    }

    // call after follower.setPose() mid-match, or the filter reads the jump as motion
    public void resetFilter() {
        distanceFilter.reset();
        goalDistance = 0;
        odometryDistance = 0;
        limelightDistance = Double.NaN;
    }

    // how much the distance (inches) changed from pedro on last update
    public double getFilterDx() {
        return distanceFilter.getDx();
    }

    // ll readings ignored in a row for being too far from the prediction
    public int getFilterRejects() {
        return distanceFilter.getRejects();
    }

    // ll frames the filter has actually blended in
    public int getFilterCorrections() {
        return distanceFilter.getCorrections();
    }

    public double getGoalDistance() {
        return goalDistance;
    }

    public double getOdometryDistance() {
        return odometryDistance;
    }

    // last limelight reading the filter was given, NaN if it hasn't had one yet
    public double getLimelightDistance() {
        return limelightDistance;
    }

    // spins up and keeps adjusting velocity to the distance from the goal
    public CommandBuilder spinUp() {
        return Commands.instant(this::startTracking);
    }


    public CommandBuilder spinUp(double velocity) {
        return Commands.instant(() -> setTargetVelocity(velocity));
    }

    public CommandBuilder spinDown() {
        return Commands.instant(this::stop);
    }

    public CommandBuilder waitUntilAtTarget() {
        return Commands.waitUntil(this::atTarget);
    }

    public void setHoodPos(double pos) {
        hood.setPosition(pos);
    }

    public void closeHood() {
        setHoodPos(closePos);
    }
}
