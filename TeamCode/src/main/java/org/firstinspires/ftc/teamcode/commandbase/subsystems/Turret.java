package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.commandbase.util.GoalAngleFilter;

import java.util.function.LongSupplier;

// 20480 ticks per turret turn (4096 * 5:1)
@Configurable
@Config
public class Turret {

    public static double ticksPerEncoderRev = 4096;
    public static double gearRatio = 5;
    public static double maxVoltage = 3.2;
    public static boolean encoderReversed = false;
    public static double encoderOffset = 0; // raw ticks when the shooter faces forward

    public static double minTicks = -10809, maxTicks = 10809; // ~ +/-190 deg

    public static double kP = 0.00035;
    public static double kD = 0.000014;
    public static double kS = 0.04;
    public static double kV = 0;           // power per tick/s of target motion
    public static double maxPower = 0.8;
    public static double wrapPower = 1;    // max power while error > wrapError, so wrapping around is fast
    public static double wrapError = 5120; // ~90 deg
    public static double tolerance = 57;   // ~1 deg

    // aiming
    public static double pivotForward = 0, pivotLeft = 0;
    public static double cameraYawOffset = 0; // deg
    public static double maxCorrectionSpeed = 3413; // ticks/s (~60 deg/s), limelight ignored above this

    private static final double TARGET_JUMP = 284; // ticks (~5 deg), a bigger target change restarts the D term

    static LongSupplier clock = System::nanoTime;

    private final DcMotorEx motor;
    private final AnalogInput encoder;

    private boolean enabled = false;
    private boolean tracking = false;
    private double target = 0;
    private double targetRate = 0;
    private boolean clamped = false;

    private double rawTicks = 0;
    private double position = 0, velocity = 0, power = 0;
    private double lastError = 0;
    private long lastNanos = 0;

    // static so it carries over from auto to teleop
    private static double encoderTicks = Double.NaN;
    private static double lastRawTicks = 0;
    private double encoderRate = 0;
    private long lastReadNanos = 0;

    private final GoalAngleFilter angleFilter = new GoalAngleFilter();
    private double odometryAngle = 0;
    private double limelightAngle = Double.NaN;
    private Pose lastAimPose = null;
    private long lastAimNanos = 0;

    public Turret(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "turret");
        encoder = hardwareMap.get(AnalogInput.class, "encoder");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lastReadNanos = clock.getAsLong();
        if (Double.isNaN(encoderTicks) || gearRatio <= 1) {
            resync();
        } else {
            rawTicks = readRawTicks();
            encoderTicks += wrap(rawTicks - lastRawTicks, ticksPerEncoderRev);
            lastRawTicks = rawTicks;
            updatePosition();
        }
        stop();
    }

    public static double ticksPerTurretRev() {
        return ticksPerEncoderRev * gearRatio;
    }

    public static double degreesToTicks(double degrees) {
        return degrees * ticksPerTurretRev() / 360;
    }

    public static double ticksToDegrees(double ticks) {
        return ticks * 360 / ticksPerTurretRev();
    }

    // any value -> (-period / 2, period / 2]
    static double wrap(double value, double period) {
        double a = value % period;
        if (a > period / 2) a -= period;
        if (a <= -period / 2) a += period;
        return a;
    }

    // encoder ticks within its own turn, 0 to ticksPerEncoderRev
    private double readRawTicks() {
        double ticks = Range.clip(encoder.getVoltage() / maxVoltage, 0, 1) * ticksPerEncoderRev;
        if (encoderReversed) ticks = ticksPerEncoderRev - ticks;
        return ticks % ticksPerEncoderRev;
    }

    // picks the encoder turn that puts the turret closest to forward
    public void resync() {
        rawTicks = readRawTicks();
        lastRawTicks = rawTicks;
        encoderTicks = encoderOffset + wrap(rawTicks - encoderOffset, ticksPerEncoderRev);
        encoderRate = 0;
        lastError = 0;
        updatePosition();
    }

    // turret is facing forward now
    public void zeroHere() {
        encoderOffset = readRawTicks();
        resync();
    }

    // hold a position in ticks, stops tracking
    public void setTarget(double ticks) {
        tracking = false;
        targetRate = 0;
        enabled = true;
        updateTarget(ticks);
    }

    public void setTargetDegrees(double degrees) {
        setTarget(degreesToTicks(degrees));
    }

    public void startTracking() {
        if (!tracking) target = position; // unwrap() starts from where the turret is
        tracking = true;
        enabled = true;
    }

    public void stop() {
        tracking = false;
        enabled = false;
        targetRate = 0;
        setPower(0);
    }

    public boolean isTracking() {
        return tracking;
    }

    public boolean isEnabled() {
        return enabled;
    }

    // call every loop while tracking, before periodic()
    public void aim(Pose robot, Alliance alliance, double limelightAngle) {
        aim(robot, Shooter.closestGoal(robot, alliance), limelightAngle);
    }

    public void aim(Pose robot, Pose goal, double limelightAngle) {
        if (!tracking) return;

        odometryAngle = angleTo(robot, goal);

        long now = clock.getAsLong();
        double dt = (now - lastAimNanos) / 1e9;
        boolean lastLoop = lastAimPose != null && dt > 0 && dt <= 0.5;
        double motion = lastLoop ? GoalAngleFilter.wrap(odometryAngle - angleTo(lastAimPose, goal)) : 0;
        lastAimPose = robot;
        lastAimNanos = now;

        // shift by this loop's motion, position is a loop old
        double measurement = Double.NaN;
        if (!Double.isNaN(limelightAngle)) {
            measurement = GoalAngleFilter.wrap(getAngle() + cameraYawOffset - limelightAngle + motion);
            if (Math.abs(velocity) > maxCorrectionSpeed) measurement = Double.NaN;
            this.limelightAngle = measurement;
        }

        targetRate = lastLoop ? degreesToTicks(motion) / dt : 0;

        updateTarget(unwrap(degreesToTicks(angleFilter.update(odometryAngle, measurement))));
    }

    // copy of the target inside the limits closest to the last target. stays on the same side until it runs
    // past a limit, then wraps a full turn. the overlap past +/-180 is hysteresis, so it can't flip back mid-wrap
    private double unwrap(double ticks) {
        double turn = ticksPerTurretRev();
        double best = Double.NaN;
        for (double c = ticks + turn * Math.ceil((minTicks - ticks) / turn); c <= maxTicks; c += turn) {
            if (Double.isNaN(best) || Math.abs(c - target) < Math.abs(best - target)) best = c;
        }
        if (!Double.isNaN(best)) return best;

        double pastMin = minTicks + wrap(ticks - minTicks, turn);
        double pastMax = maxTicks + wrap(ticks - maxTicks, turn);
        return Math.abs(pastMin - minTicks) < Math.abs(pastMax - maxTicks) ? pastMin : pastMax;
    }

    // deg, robot frame
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

        long now = clock.getAsLong();
        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;
        if (dt <= 0 || dt > 0.5) dt = 0;

        double error = target - position;

        double derivative = dt > 0 ? (error - lastError) / dt : 0;
        lastError = error;

        double out = kP * error + kD * derivative + kV * targetRate;
        if (Math.abs(error) > tolerance) out += kS * Math.signum(error);
        double limit = Math.abs(error) > wrapError ? wrapPower : maxPower;
        out = Range.clip(out, -limit, limit);

        if ((position >= maxTicks && out > 0) || (position <= minTicks && out < 0)) out = 0;

        setPower(out);
    }

    private void updateTarget(double ticks) {
        double clipped = Range.clip(ticks, minTicks, maxTicks);
        clamped = clipped != ticks;

        if (Math.abs(clipped - target) > TARGET_JUMP) lastError = clipped - position;
        target = clipped;
    }

    // counts encoder turns, assuming it keeps its last speed
    private void readEncoder() {
        long now = clock.getAsLong();
        double dt = (now - lastReadNanos) / 1e9;
        lastReadNanos = now;
        boolean valid = dt > 0 && dt <= 0.5;

        rawTicks = readRawTicks();
        double predicted = valid ? encoderRate * dt : 0;
        double delta = predicted + wrap(rawTicks - lastRawTicks - predicted, ticksPerEncoderRev);
        encoderTicks += delta;
        lastRawTicks = rawTicks;
        encoderRate = valid ? delta / dt : 0;

        updatePosition();
    }

    private void updatePosition() {
        position = encoderTicks - encoderOffset;
        velocity = encoderRate;
    }

    private void setPower(double p) {
        power = p;
        motor.setPower(p);
    }

    // encoder ticks within its own turn
    public double getRawTicks() {
        return rawTicks;
    }

    public double getVoltage() {
        return encoder.getVoltage();
    }

    // ticks from forward
    public double getPosition() {
        return position;
    }

    // deg from forward
    public double getAngle() {
        return ticksToDegrees(position);
    }

    // ticks/s
    public double getVelocity() {
        return velocity;
    }

    public double getTarget() {
        return target;
    }

    public double getTargetDegrees() {
        return ticksToDegrees(target);
    }

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

    public boolean isClamped() {
        return clamped;
    }

    public boolean atTarget() {
        return enabled && !clamped && Math.abs(getError()) < tolerance;
    }

    // call after follower.setPose() or a heading reset
    public void resetFilter() {
        angleFilter.reset();
        odometryAngle = 0;
        limelightAngle = Double.NaN;
        lastAimPose = null;
        targetRate = 0;
    }

    // deg
    public double getOdometryAngle() {
        return odometryAngle;
    }

    // deg
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

    public CommandBuilder toTarget(double ticks) {
        return Commands.instant(() -> setTarget(ticks));
    }

    public CommandBuilder toTargetDegrees(double degrees) {
        return Commands.instant(() -> setTargetDegrees(degrees));
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
