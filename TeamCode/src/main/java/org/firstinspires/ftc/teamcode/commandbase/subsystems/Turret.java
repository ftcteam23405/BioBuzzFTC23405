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
import org.firstinspires.ftc.teamcode.commandbase.util.GoalAngleFilter;


// every position, target, limit and PID number in here is in ENCODER TICKS from zero (0 = shooter facing forward,
// positive = left). degrees only show up in the aiming math (pedro + limelight), and get turned into ticks with
// degreesToTicks() before they reach the PID. to command a degree, pass degreesToTicks(deg)
// ~2.02 ticks per turret deg, ~725.5 ticks per turret turn

@Configurable
@Config
public class Turret {

    // encoder
    public static double motorTicksPerRev = 145.1; // goBILDA 1150 rpm yellow jacket (5.2:1): 28 ticks * 5.2 at the output shaft
    public static double gearRatio = 5;
                                                   // = 725.5 ticks per turret turn, ~2.02 ticks per deg

    // ticks. target is clamped to these. they can go past +/- half a turn (+/-363 ticks) if the wiring allows it,
    // tracking wraps the goal into this range and unwinds when it runs out
    public static double minTicks = -302, maxTicks = 302; // ~ +/-150 deg

    // pos PID, all per tick of error
    public static double kP = 0.0099;
    public static double kI = 0;
    public static double kD = 0.0004;

    public static double kS = 0.04;     // power to overcome friction
    public static double kV = 0;        // power per tick/s the target is moving (robot turning), helps it keep up due to robot velocity
    public static double iZone = 10;    // ticks, the integral only builds up inside this much error
    public static double maxPower = 0.8;
    public static double tolerance = 2; // ticks

    // aiming
    public static double pivotForward = 0, pivotLeft = 0;
    public static double cameraYawOffset = 0;
    public static double maxCorrectionSpeed = 121; // ticks/s (~60 deg/s). ignore the limelight while the turret slews faster than this, tx is a frame behind and would be blended against the wrong turret position

    private final DcMotorEx motor;

    private boolean enabled = false;   // false = motor off, true = PID holds target
    private boolean tracking = false;  // true = target follows the goal every loop
    private double target = 0;         // ticks
    private double targetRate = 0;     // ticks/s, from pedro, for kV
    private boolean clamped = false;   // the goal is past a travel limit, so the turret is parked at the limit

    // cached in periodic()
    private int rawTicks = 0;
    private double position = 0, velocity = 0, power = 0; // ticks from zero, ticks/s
    private double lastError = 0, integral = 0;
    private long lastNanos = 0;

    // raw encoder ticks when the shooter faces forward. static so it carries over from auto to teleop,
    // the hub doesn't reset the count between opmodes. NaN until the first Turret is made after the app starts
    private static double zeroTicks = Double.NaN;

    // pedro pose + limelight = filtered turret angle (deg) to the closest goal center
    private final GoalAngleFilter angleFilter = new GoalAngleFilter();
    private double odometryAngle = 0;
    private double limelightAngle = Double.NaN;
    private double lastOdometryAngle = Double.NaN;
    private long lastAimNanos = 0;

    public Turret(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "turret");


        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Double.isNaN(zeroTicks)) zeroTicks = motor.getCurrentPosition();
        readEncoder();
        stop();
    }


    public static double degreesToTicks(double degrees) {
        return degrees * motorTicksPerRev * gearRatio / 360;
    }

    // ticks -> deg, for telemetry and the aiming math
    public static double ticksToDegrees(double ticks) {
        return ticks * 360 / (motorTicksPerRev * gearRatio);
    }

    // wherever the turret is now becomes 0 ticks. point the shooter straight forward first
    public void resetPosition() {
        resetPosition(0);
    }

    // wherever the turret is now becomes this many ticks, e.g. pushed against a hard stop at a known position
    public void resetPosition(double ticks) {
        zeroTicks = motor.getCurrentPosition() - ticks;
        lastError = 0;
        integral = 0;
        readEncoder();
    }

    // hold a fixed position in ticks, stops tracking
    public void setTarget(double ticks) {
        tracking = false;
        targetRate = 0;
        enabled = true;
        updateTarget(ticks);
    }

    // aim at the closest goal until stop() or setTarget()
    // the filter isn't reset here, it keeps the drift correction it learned from the limelight
    public void startTracking() {
        tracking = true;
        enabled = true;
    }

    // motor off, the turret can be turned by hand
    public void stop() {
        tracking = false;
        enabled = false;
        targetRate = 0;
        integral = 0;
        setPower(0);
    }

    public boolean isTracking() {
        return tracking;
    }

    public boolean isEnabled() {
        return enabled;
    }

    // works out where to point at our alliance's closest goal. call every loop while tracking, before periodic()
    // don't pass the turret's own angle here or the same frame gets blended in over and over
    public void aim(Pose robot, Alliance alliance, double limelightAngle) {
        aim(robot, Shooter.closestGoal(robot, alliance), limelightAngle);
    }

    // same, at a goal that's already been picked (Robot picks it once per loop for the shooter and turret)
    // limelightAngle (deg) must come from that goal's tags, see Limelight.freshAngle(int[])
    // switching goals is fine, the filter moves with pedro's jump in angle and keeps its limelight correction
    public void aim(Pose robot, Pose goal, double limelightAngle) {
        if (!tracking) return;

        odometryAngle = angleTo(robot, goal);

        // the limelight sees the goal tx deg to the right of the camera, which points cameraYawOffset left of
        // the shooter. position is from the last loop, which is closer to when the frame was taken anyway
        double measurement = Double.NaN;
        if (!Double.isNaN(limelightAngle)) {
            measurement = GoalAngleFilter.wrap(ticksToDegrees(position) + cameraYawOffset - limelightAngle);
            this.limelightAngle = measurement;
            if (Math.abs(velocity) > maxCorrectionSpeed) measurement = Double.NaN;
        }

        // how fast the goal is moving across the robot frame, mostly the robot turning
        long now = System.nanoTime();
        double dt = (now - lastAimNanos) / 1e9;
        targetRate = (Double.isNaN(lastOdometryAngle) || dt <= 0 || dt > 0.5)
                ? 0 : degreesToTicks(GoalAngleFilter.wrap(odometryAngle - lastOdometryAngle)) / dt;
        lastOdometryAngle = odometryAngle;
        lastAimNanos = now;

        updateTarget(unwrap(degreesToTicks(angleFilter.update(odometryAngle, measurement))));
    }

    // the filter's angle is wrapped to (-180, 180] deg, but the turret points the same way a whole turn either side.
    // picks the copy inside the travel limits closest to where the turret is now, so it takes the short way round
    // when it can and unwinds the long way when the short way is past a limit. if no copy is inside the limits,
    // returns the one just past the closest limit, and updateTarget() parks the turret at that limit
    private double unwrap(double ticks) {
        double turn = degreesToTicks(360);
        double best = Double.NaN;
        for (double c = ticks + turn * Math.ceil((minTicks - ticks) / turn); c <= maxTicks; c += turn) {
            if (Double.isNaN(best) || Math.abs(c - position) < Math.abs(best - position)) best = c;
        }
        if (!Double.isNaN(best)) return best;

        double pastMin = minTicks + wrapTicks(ticks - minTicks);
        double pastMax = maxTicks + wrapTicks(ticks - maxTicks);
        return Math.abs(pastMin - minTicks) < Math.abs(pastMax - maxTicks) ? pastMin : pastMax;
    }

    // any tick offset -> within half a turn either side of 0
    private static double wrapTicks(double ticks) {
        return degreesToTicks(GoalAngleFilter.wrap(ticksToDegrees(ticks)));
    }

    // turret angle in deg (robot frame) that points the shooter from the turret pivot at the goal
    public static double angleTo(Pose robot, Pose goal) {
        double heading = robot.heading();
        double pivotX = robot.x() + pivotForward * Math.cos(heading) - pivotLeft * Math.sin(heading);
        double pivotY = robot.y() + pivotForward * Math.sin(heading) + pivotLeft * Math.cos(heading);

        double fieldAngle = Math.atan2(goal.y() - pivotY, goal.x() - pivotX);
        return GoalAngleFilter.wrap(Math.toDegrees(fieldAngle - heading));
    }

    public void periodic() {
        readEncoder();

        if (!enabled) {
            setPower(0);
            return;
        }

        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;
        if (dt <= 0 || dt > 0.5) dt = 0; // first loop, or the loop stalled: skip the I and D for this one

        // both are unwrapped (the encoder counts straight through half a turn), unwrap() already picked which way round
        double error = target - position;

        if (kI != 0 && Math.abs(error) < iZone) integral += error * dt;
        else integral = 0;

        double derivative = dt > 0 ? (error - lastError) / dt : 0;
        lastError = error;

        double out = kP * error + kI * integral + kD * derivative + kV * targetRate;
        if (Math.abs(error) > tolerance) out += kS * Math.signum(error);
        out = Range.clip(out, -maxPower, maxPower);

        // never push further past a limit (overshoot, or the turret wasn't forward when it was zeroed)
        if ((position >= maxTicks && out > 0) || (position <= minTicks && out < 0)) out = 0;

        setPower(out);
    }

    // ticks, clamped to the travel limits
    private void updateTarget(double ticks) {
        double clipped = Range.clip(ticks, minTicks, maxTicks);
        clamped = clipped != ticks;

        // a new target would show up as a huge derivative, start the D term from the new error
        if (Math.abs(clipped - target) > iZone) {
            lastError = clipped - position;
            integral = 0;
        }
        target = clipped;
    }

    // one encoder read per loop, position and velocity both come out of the same bulk read
    private void readEncoder() {
        rawTicks = motor.getCurrentPosition();
        position = rawTicks - zeroTicks;
        velocity = motor.getVelocity(); // ticks/s, the hub works this out from the encoder
    }

    private void setPower(double p) {
        power = p;
        motor.setPower(p);
    }

    // raw encoder count, 0 ticks is at getZeroTicks()
    public int getRawTicks() {
        return rawTicks;
    }

    public double getZeroTicks() {
        return zeroTicks;
    }

    // ticks from zero
    public double getPosition() {
        return position;
    }

    // deg, for telemetry. same as ticksToDegrees(getPosition())
    public double getAngle() {
        return ticksToDegrees(position);
    }

    // ticks/s
    public double getVelocity() {
        return velocity;
    }

    // ticks
    public double getTarget() {
        return target;
    }

    // ticks
    public double getError() {
        return target - position;
    }

    public double getPower() {
        return power;
    }

    // ticks/s
    public double getTargetRate() {
        return targetRate;
    }

    // true when the goal is past a travel limit and the turret is parked at the limit instead
    public boolean isClamped() {
        return clamped;
    }

    // pointed where it wants to be. false while clamped, it's at a limit, not on the goal
    public boolean atTarget() {
        return enabled && !clamped && Math.abs(getError()) < tolerance;
    }

    // call after follower.setPose() or a heading reset mid-match, or the filter reads the jump as the robot turning
    public void resetFilter() {
        angleFilter.reset();
        odometryAngle = 0;
        limelightAngle = Double.NaN;
        lastOdometryAngle = Double.NaN;
        targetRate = 0;
    }

    // turret angle (deg) to the goal from pedro alone
    public double getOdometryAngle() {
        return odometryAngle;
    }

    // the last turret angle (deg) to the goal from the limelight, NaN if it hasn't had one yet
    public double getLimelightAngle() {
        return limelightAngle;
    }

    public double getFilterDx() {
        return angleFilter.getDx();
    }

    public int getFilterRejects() {
        return angleFilter.getRejects();
    }

    public int getFilterCorrections() {
        return angleFilter.getCorrections();
    }

    public CommandBuilder track() {
        return Commands.instant(this::startTracking);
    }

    // ticks
    public CommandBuilder toTarget(double ticks) {
        return Commands.instant(() -> setTarget(ticks));
    }

    public CommandBuilder center() {
        return toTarget(0);
    }

    public CommandBuilder off() {
        return Commands.instant(this::stop);
    }

    public CommandBuilder waitUntilAtTarget() {
        return Commands.waitUntil(this::atTarget);
    }
}
