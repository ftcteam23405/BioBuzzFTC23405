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


// position comes from absolute analog output : 0-3.2V = 0-360 deg of the
// encoder, so turret deg = volts * degreesPerVolt()

@Configurable
@Config
public class Turret {

    // encoder
    public static double maxVoltage = 3.2;         // analog output at 360 deg, 3.2V on a 3.3V hub per the melonbotics docs
    public static double gearRatio = 125.0 / 25.0;
                                                   // turret has to start within +/-36 deg of forward
    public static boolean encoderReversed = false; // flip if angle goes down when the turret turns left
    public static double encoderOffset = 0;        // raw degrees when the shooter faces the robot's front


    public static boolean motorReversed = false;

    // target is clamped to these angles
    public static double minAngle = -150, maxAngle = 150;

    // pos PID
    public static double kP = 0.02;
    public static double kI = 0;
    public static double kD = 0.0008;

    public static double kS = 0.04;     // power to overcome friction
    public static double kV = 0;        // power per deg/s the target is moving (robot turning), helps it keep up due to robot velocity
    public static double iZone = 5;     // deg, the integral only builds up inside this much error
    public static double maxPower = 0.8;
    public static double tolerance = 1; // in deg

    // aiming
    public static double pivotForward = 0, pivotLeft = 0; // turret pivot from the robot center (in), robot frame
    public static double cameraYawOffset = 0;  // deg the limelight points left of where the shooter shoots
    public static double maxCorrectionSpeed = 60; // deg/s. ignore the limelight while the turret slews faster than this,
                                                  // tx is a frame behind and would be blended against the wrong turret angle

    private final DcMotorEx motor;
    private final AnalogInput encoder;

    private boolean enabled = false;   // false = motor off, true = PID holds targetAngle
    private boolean tracking = false;  // true = targetAngle follows the goal every loop
    private double targetAngle = 0;
    private double targetRate = 0;     // deg/s, from pedro, for kV
    private boolean clamped = false;   // the goal is past a travel limit, so the turret is parked at the limit

    // cached in periodic()
    private double voltage = 0, angle = 0, velocity = 0, power = 0;
    private double lastError = 0, integral = 0;
    private long lastNanos = 0, lastReadNanos = 0;

    // encoder turns counted past the seam, only ever nonzero when gearRatio isn't 1
    private int wraps = 0;
    private double lastEncoderAngle = Double.NaN;

    // static so it carries over from auto to teleop: the new Turret works out its wrap count from where the
    // last one left off, instead of assuming the turret is near forward. back to 0 if the app restarts
    private static double lastKnownAngle = 0;

    // pedro pose + limelight = filtered turret angle to the closest goal center
    private final GoalAngleFilter angleFilter = new GoalAngleFilter();
    private double odometryAngle = 0;
    private double limelightAngle = Double.NaN;
    private double lastOdometryAngle = Double.NaN;
    private long lastAimNanos = 0;

    public Turret(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        encoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        motor.setDirection(motorReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetWraps(lastKnownAngle);
        stop();
    }

    // works the wrap count out again, picking whichever one puts the turret closest to nearAngle
    // the encoder alone can't tell which of its 5 turns it's on, so this is only right if the turret is
    // within +/-180 / gearRatio (36) deg of nearAngle. call it with the turret still
    public void resetWraps(double nearAngle) {
        voltage = encoder.getVoltage();
        double encoderAngle = GoalAngleFilter.wrap(getRawDegrees() - encoderOffset);

        // turret angle = (encoderAngle + 360 * wraps) / gearRatio, solved for the wraps closest to nearAngle
        wraps = (int) Math.round((nearAngle * gearRatio - encoderAngle) / 360);
        lastEncoderAngle = encoderAngle;
        angle = (encoderAngle + 360 * wraps) / gearRatio;
        lastKnownAngle = angle;

        velocity = 0;
        lastReadNanos = 0;
    }

    // hold a fixed angle, stops tracking
    public void setTargetAngle(double degrees) {
        tracking = false;
        targetRate = 0;
        enabled = true;
        setTarget(degrees);
    }

    // aim at the closest goal until stop() or setTargetAngle()
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

    // works out where to point. call every loop while tracking, before periodic()
    // don't pass angle() here or the same frame gets blended in over and over
    public void aim(Pose robot, Alliance alliance, double limelightAngle) {
        if (!tracking) return;

        Pose goal = Shooter.closestGoal(robot, alliance);
        odometryAngle = angleTo(robot, goal);

        // the limelight sees the goal tx deg to the right of the camera, which points cameraYawOffset left of
        // the shooter. angle is from the last loop, which is closer to when the frame was taken anyway
        double measurement = Double.NaN;
        if (!Double.isNaN(limelightAngle)) {
            measurement = GoalAngleFilter.wrap(angle + cameraYawOffset - limelightAngle);
            this.limelightAngle = measurement;
            if (Math.abs(velocity) > maxCorrectionSpeed) measurement = Double.NaN;
        }

        // how fast the goal is moving across the robot frame, mostly the robot turning
        long now = System.nanoTime();
        double dt = (now - lastAimNanos) / 1e9;
        targetRate = (Double.isNaN(lastOdometryAngle) || dt <= 0 || dt > 0.5)
                ? 0 : GoalAngleFilter.wrap(odometryAngle - lastOdometryAngle) / dt;
        lastOdometryAngle = odometryAngle;
        lastAimNanos = now;

        setTarget(angleFilter.update(odometryAngle, measurement));
    }

    // turret angle (robot frame) that points the shooter from the turret pivot at the goal
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

        // no wrap needed, the turret can't go around the back so the short way is the only way
        double error = targetAngle - angle;

        if (kI != 0 && Math.abs(error) < iZone) integral += error * dt;
        else integral = 0;

        double derivative = dt > 0 ? (error - lastError) / dt : 0;
        lastError = error;

        double out = kP * error + kI * integral + kD * derivative + kV * targetRate;
        if (Math.abs(error) > tolerance) out += kS * Math.signum(error);
        out = Range.clip(out, -maxPower, maxPower);

        // never push further past a limit (overshoot, or encoderOffset is wrong)
        if ((angle >= maxAngle && out > 0) || (angle <= minAngle && out < 0)) out = 0;

        setPower(out);
    }

    private void setTarget(double degrees) {
        double clipped = Range.clip(degrees, minAngle, maxAngle);
        clamped = clipped != degrees;

        // a new target would show up as a huge derivative, start the D term from the new error
        if (Math.abs(clipped - targetAngle) > iZone) {
            lastError = clipped - angle;
            integral = 0;
        }
        targetAngle = clipped;
    }

    // one encoder read per loop
    private void readEncoder() {
        voltage = encoder.getVoltage();

        // encoder angle in (-180, 180]. jumping more than half a turn in one loop means it crossed the seam
        double encoderAngle = GoalAngleFilter.wrap(getRawDegrees() - encoderOffset);
        if (!Double.isNaN(lastEncoderAngle)) {
            if (encoderAngle - lastEncoderAngle > 180) wraps--;
            else if (encoderAngle - lastEncoderAngle < -180) wraps++;
        }
        lastEncoderAngle = encoderAngle;

        double newAngle = (encoderAngle + 360 * wraps) / gearRatio;

        long now = System.nanoTime();
        double dt = (now - lastReadNanos) / 1e9;
        if (lastReadNanos != 0 && dt > 0 && dt < 0.5)
            velocity = 0.7 * velocity + 0.3 * (newAngle - angle) / dt; // a little smoothing, the analog signal is noisy
        lastReadNanos = now;

        angle = newAngle;
        lastKnownAngle = angle;
    }

    // turret deg per volt of encoder output
    public static double degreesPerVolt() {
        return 360 / maxVoltage / gearRatio;
    }

    private void setPower(double p) {
        power = p;
        motor.setPower(p);
    }

    // encoder angle before encoderOffset, 0..360 of the ENCODER, not the turret
    // read this with the shooter facing forward to set encoderOffset
    public double getRawDegrees() {
        double deg = Range.clip(voltage / maxVoltage, 0, 1) * 360;
        return encoderReversed ? 360 - deg : deg;
    }

    public double getVoltage() {
        return voltage;
    }

    public int getWraps() {
        return wraps;
    }

    public double getAngle() {
        return angle;
    }

    // deg/s, smoothed
    public double getVelocity() {
        return velocity;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getError() {
        return targetAngle - angle;
    }

    public double getPower() {
        return power;
    }

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

    // turret angle to the goal from pedro alone
    public double getOdometryAngle() {
        return odometryAngle;
    }

    // the last turret angle to the goal from the limelight, NaN if it hasn't had one yet
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

    public CommandBuilder toAngle(double degrees) {
        return Commands.instant(() -> setTargetAngle(degrees));
    }

    public CommandBuilder center() {
        return toAngle(0);
    }

    public CommandBuilder off() {
        return Commands.instant(this::stop);
    }

    public CommandBuilder waitUntilAtTarget() {
        return Commands.waitUntil(this::atTarget);
    }
}
