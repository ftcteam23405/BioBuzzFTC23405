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

import org.firstinspires.ftc.teamcode.commandbase.util.GoalAngleFilter;

import java.util.function.LongSupplier;

// every position, target, limit and PID number in here is in MOTOR ENCODER TICKS from forward
// (0 = shooter facing forward, positive = left / ccw from above). 145.1 * 5 = 725.5 ticks per turret turn,
// ~2.02 ticks per deg. degrees only show up in the aiming math and get turned into ticks with degreesToTicks()
//
// the analog absolute encoder is ONLY used to find where the turret is when the code starts (resync()).
// after that everything runs off the motor's own encoder, which the hub keeps counting between opmodes,
// so the zero is static and carries over from auto to teleop
@Configurable
@Config
public class Turret {

    // motor encoder (all the control math)
    public static double motorTicksPerRev = 145.1; // goBILDA 1150 rpm yellow jacket, at the output shaft
    public static double gearRatio = 5;            // motor output turns per turret turn
    public static boolean motorReversed = false;   // flips power AND the encoder together, so the PID can't fight itself

    // absolute encoder (only for the starting position)
    public static double minVoltage = 0, maxVoltage = 3.2; // lowest / highest voltage it reads over one of its turns
    public static boolean encoderReversed = false;
    public static double encoderGearRatio = 5; // abs encoder turns per turret turn (5 on the 5:1 input, 1 on the turret)
    public static double encoderOffset = 0;    // abs encoder reading (0-360 deg) when the shooter faces forward

    // ticks. +/-180 deg
    public static double minTicks = -362, maxTicks = 362;
    public static double wrapBuffer = 10; // ticks (~5 deg) the goal can go past a limit before the turret wraps

    // starting gains from a physics sim (estimated friction / inertia / backlash), confirm on the robot
    public static double kP = 0.02;
    public static double kD = 0.0016;
    public static double kS = 0.08;
    public static double kV = 0.0003;      // power per tick/s of target motion, ~1 / turret top speed
    public static double maxPower = 0.8;
    public static double wrapPower = 1;    // max power while error > wrapError, so wrapping around is fast
    public static double wrapError = 181;  // ~90 deg
    public static double tolerance = 2;    // ~1 deg

    // aiming
    public static double pivotForward = 0, pivotLeft = 0;
    public static double cameraYawOffset = 0; // deg
    public static double maxCorrectionSpeed = 121; // ticks/s (~60 deg/s), limelight ignored above this

    static LongSupplier clock = System::nanoTime;

    private final DcMotorEx motor;
    private final AnalogInput encoder;

    private boolean enabled = false;
    private boolean tracking = false;
    private double target = 0;
    private double targetRate = 0;
    private boolean clamped = false;

    private double position = 0, velocity = 0, power = 0;
    private double voltage = 0, encoderDegrees = 0;

    // raw motor encoder count (direction not applied) when the shooter faces forward. static so it carries over
    // from auto to teleop, NaN until the first Turret after the app starts
    private static double zeroTicks = Double.NaN;

    private final GoalAngleFilter angleFilter = new GoalAngleFilter();
    private double odometryAngle = 0;
    private double limelightAngle = Double.NaN;
    private Pose lastAimPose = null;
    private long lastAimNanos = 0;

    public Turret(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "turret");
        encoder = hardwareMap.get(AnalogInput.class, "encoder");

        // keep this FORWARD, motorReversed does the flipping so it can be changed live
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // doesn't reset the count
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Double.isNaN(zeroTicks)) resync();
        else readMotor();
        stop();
    }

    public static double ticksPerTurretRev() {
        return motorTicksPerRev * gearRatio;
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

    private double direction() {
        return motorReversed ? -1 : 1;
    }

    private double rawMotorTicks() {
        return motor.getCurrentPosition();
    }

    // ---------------- absolute encoder ----------------

    // reads the abs encoder once and caches it. only resync() and testers need this, not the control loop
    public void readAbsolute() {
        voltage = encoder.getVoltage();
        double span = Math.max(maxVoltage - minVoltage, 1e-3);
        double deg = Range.clip((voltage - minVoltage) / span, 0, 1) * 360;
        if (encoderReversed) deg = 360 - deg;
        encoderDegrees = deg % 360;
    }

    // turret deg from forward according to the abs encoder alone. only right within +/- 180 / encoderGearRatio
    // of forward (+/-36 deg at 5:1), past that it repeats
    public double getAbsoluteAngle() {
        return GoalAngleFilter.wrap(encoderDegrees - encoderOffset) / encoderGearRatio;
    }

    // turret deg the motor encoder and the abs encoder disagree by (mod one abs encoder turn). should stay ~0
    // as the turret moves anywhere. if it grows as you turn, motorTicksPerRev / gearRatio / encoderGearRatio
    // is wrong. if it jumps, a belt or gear slipped
    public double getAbsoluteCheck() {
        double expected = encoderOffset + getAngle() * encoderGearRatio;
        return GoalAngleFilter.wrap(encoderDegrees - expected) / encoderGearRatio;
    }

    // sets the motor encoder zero from the abs encoder, assuming the turret is within +/-36 deg (at 5:1) of forward
    public void resync() {
        readAbsolute();
        zeroTicks = rawMotorTicks() - direction() * degreesToTicks(getAbsoluteAngle());
        readMotor();
    }

    // turret is facing forward now
    public void zeroHere() {
        readAbsolute();
        encoderOffset = encoderDegrees;
        resync();
    }

    // ---------------- commands ----------------

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

    // ---------------- aiming ----------------

    // call every loop while tracking, before periodic()
    public void aim(Pose robot, double limelightAngle) {
        aim(robot, Shooter.closestGoal(robot), limelightAngle);
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

    // copy of the target closest to the last target, within the limits +/- wrapBuffer. stays on the same side
    // (held at the limit by updateTarget) until the goal is wrapBuffer past a limit, then wraps a full turn.
    // the buffer is so it can't flip back and forth when the goal is right behind the robot
    private double unwrap(double ticks) {
        double turn = ticksPerTurretRev();
        double min = minTicks - wrapBuffer, max = maxTicks + wrapBuffer;
        double best = Double.NaN;
        for (double c = ticks + turn * Math.ceil((min - ticks) / turn); c <= max; c += turn) {
            if (Double.isNaN(best) || Math.abs(c - target) < Math.abs(best - target)) best = c;
        }
        if (!Double.isNaN(best)) return best;

        // limits less than a turn apart and the goal is in the gap between them: stay at the limit we're already
        // at. picking the limit nearest the goal would flip sides every time the goal crosses the middle of the gap
        return Math.abs(target - minTicks) < Math.abs(target - maxTicks) ? minTicks : maxTicks;
    }

    // deg, robot frame
    public static double angleTo(Pose robot, Pose goal) {
        double heading = robot.heading();
        double pivotX = robot.x() + pivotForward * Math.cos(heading) - pivotLeft * Math.sin(heading);
        double pivotY = robot.y() + pivotForward * Math.sin(heading) + pivotLeft * Math.cos(heading);

        double fieldAngle = Math.atan2(goal.y() - pivotY, goal.x() - pivotX);
        return GoalAngleFilter.wrap(Math.toDegrees(fieldAngle - heading));
    }

    // ---------------- control loop ----------------

    public void periodic() {
        readMotor();

        if (!enabled) {
            setPower(0);
            return;
        }

        double error = target - position;

        // d(error)/dt from the hub's measured velocity instead of differencing the error, so target jumps
        // (setTarget, limelight corrections) don't kick it
        double derivative = targetRate - velocity;

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
        target = clipped;
    }

    private void readMotor() {
        position = direction() * (rawMotorTicks() - zeroTicks);
        velocity = direction() * motor.getVelocity();
    }

    private void setPower(double p) {
        power = p;
        motor.setPower(direction() * p);
    }

    // ---------------- getters ----------------

    // raw motor encoder count, no zero or direction applied
    public double getMotorTicks() {
        return rawMotorTicks();
    }

    // from the last readAbsolute()
    public double getVoltage() {
        return voltage;
    }

    // abs encoder reading 0-360 deg, from the last readAbsolute()
    public double getEncoderDegrees() {
        return encoderDegrees;
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
