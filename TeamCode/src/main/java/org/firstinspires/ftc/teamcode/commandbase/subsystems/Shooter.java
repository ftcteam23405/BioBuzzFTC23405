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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.commandbase.util.GoalDistanceFilter;

import smile.interpolation.Interpolation;
import smile.interpolation.LinearInterpolation;

@Configurable
@Config
public class Shooter {


    public static double kS = 0.08;      // power to overcome friction
    public static double kV = 0.00039;   // power per tick/s
    public static double kP = 0.01;      // power per tick/s of error
    public static double tolerance = 50; // ticks/s


    public static double redGoalX = 58;
    public static double blueGoalX = 84;
    public static double audienceGoalY = 52;
    public static double farGoalY = 90;

    // pedro + limelight are combined in GoalDistanceFilter, tune it with GoalDistanceFilterTuner

    // straight-line distance to the closest goal (inches) -> flywheel velocity (ticks/s)
    // placeholder values, fill these in with ShooterTuner. distances must be increasing
    private static final double[] DISTANCES = {40, 60, 80, 100, 120};
    private static final double[] VELOCITIES = {1200, 1250, 1300, 1350, 1400};
    private static final Interpolation velocityTable = new LinearInterpolation(DISTANCES, VELOCITIES);

    private final DcMotorEx topMotor, bottomMotor;

    private double targetVelocity = 0;
    private boolean tracking = false; // true = velocity follows distance to the goal every loop

    // pedro pose + limelight -> filtered straight-line distance from the robot to the closest goal center
    private final GoalDistanceFilter distanceFilter = new GoalDistanceFilter();
    private double goalDistance = 0;

    // raw readings from the last update, for telemetry
    private double odometryDistance = 0;
    private double limelightDistance = Double.NaN;

    public Shooter(HardwareMap hardwareMap) {
        topMotor = hardwareMap.get(DcMotorEx.class, "shooterMotorTop");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "shooterMotorBottom");

        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        stop();
    }

    // spin at a fixed velocity, stops distance tracking
    public void setTargetVelocity(double velocity) {
        tracking = false;
        targetVelocity = Math.max(0, velocity);
    }

    // velocity follows the distance to the goal until stop() or setTargetVelocity()
    // the filter isn't reset here: it still has the last pedro pose, so it picks up where it left off
    // and keeps the drift correction it learned from the limelight
    public void startTracking() {
        tracking = true;
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
        return topMotor.getVelocity();
    }

    public boolean atTarget() {
        return (targetVelocity > 0) && Math.abs(targetVelocity - getVelocity()) < tolerance;
    }

    public void periodic() {
        if (targetVelocity == 0) {
            setPower(0);
            return;
        }

        double feedforward = kS + kV * targetVelocity;
        double feedback = kP * (targetVelocity - getVelocity());

        // don't reverse the flywheel to slow it down, just let it coast
        setPower(Range.clip(feedforward + feedback, 0, 1));
    }

    private void setPower(double power) {
        topMotor.setPower(power);
        bottomMotor.setPower(power);
    }


    // sets the target velocity from how far we are from the goal, using odometry + limelight
    // robot: follower pose
    // limelightDistance: from Limelight.freshDistanceToGoal(), which is NaN unless the limelight gave us a
    // frame we haven't used yet. don't pass distanceToGoal() here, that reading repeats between frames and
    // the filter would blend the same number in over and over
    public void setVelocityFromDistance(Pose robot, Alliance alliance, double limelightDistance) {
        Pose goal = closestGoal(robot, alliance);
        odometryDistance = robot.distance(goal);
        if (!Double.isNaN(limelightDistance)) this.limelightDistance = limelightDistance;

        // pedro dx moves the estimate, the limelight pulls it back when a tag is visible
        goalDistance = distanceFilter.update(robot, goal, limelightDistance);
        targetVelocity = velocityFor(goalDistance);
    }

    // sets the target velocity from a limelight distance only (no pedro), still filtered
    public void setVelocityFromDistance(double distance) {
        if (Double.isNaN(distance)) return; // keep the last target instead of spinning at garbage

        this.limelightDistance = distance;
        goalDistance = distanceFilter.update(distance);
        targetVelocity = velocityFor(goalDistance);
    }

    // looks up the velocity for a distance from the goal, clamped to the ends of the table
    public static double velocityFor(double distance) {
        double d = Range.clip(distance, DISTANCES[0], DISTANCES[DISTANCES.length - 1]);
        return velocityTable.interpolate(d);
    }

    // the one of our 2 cells that's closer to the robot
    public static Pose closestGoal(Pose robot, Alliance alliance) {
        double goalX = alliance == Alliance.RED ? redGoalX : blueGoalX;
        Pose audienceGoal = new Pose(goalX, audienceGoalY);
        Pose farGoal = new Pose(goalX, farGoalY);

        return robot.distance(audienceGoal) < robot.distance(farGoal) ? audienceGoal : farGoal;
    }

    // call after follower.setPose() mid-match, or the filter reads the jump as motion
    public void resetFilter() {
        distanceFilter.reset();
        goalDistance = 0;
        odometryDistance = 0;
        limelightDistance = Double.NaN;
    }

    // how much the distance changed from pedro on the last update (inches)
    public double getFilterDx() {
        return distanceFilter.getDx();
    }

    // limelight readings ignored in a row for being too far from the prediction
    public int getFilterRejects() {
        return distanceFilter.getRejects();
    }

    // limelight frames the filter has actually blended in
    public int getFilterCorrections() {
        return distanceFilter.getCorrections();
    }

    public double getGoalDistance() {
        return goalDistance;
    }

    public double getOdometryDistance() {
        return odometryDistance;
    }

    // the last limelight reading the filter was given, NaN if it hasn't had one yet
    public double getLimelightDistance() {
        return limelightDistance;
    }

    // spins up and keeps adjusting velocity to the distance from the goal
    public CommandBuilder spinUp() {
        return Commands.instant(this::startTracking);
    }

    // spins up to a fixed velocity
    public CommandBuilder spinUp(double velocity) {
        return Commands.instant(() -> setTargetVelocity(velocity));
    }

    public CommandBuilder spinDown() {
        return Commands.instant(this::stop);
    }

    public CommandBuilder waitUntilAtTarget() {
        return Commands.waitUntil(this::atTarget);
    }
}
