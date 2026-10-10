package org.firstinspires.ftc.teamcode.commandbase.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

// complementary filter for the turret angle that points the shooter at the goal center
// same idea as GoalDistanceFilter, but for an angle instead of a distance
//
// every loop:
//   1. predict: dθ = (turret angle pedro says points at the goal NOW) - (what it said LAST loop)
//               x̂ = x̂ + dθ
//   2. correct: only when the limelight has a NEW frame (y = turret angle + camera yaw - tx),
//               x̂ = x̂ + limelightWeight * (y - x̂)
//
// all angles are degrees in the robot frame: 0 = robot front, positive = left (ccw from above)
// the odometry angle already has the robot heading taken out of it, so dθ is mostly the robot turning,
// and the turret counter-rotates by exactly that much between limelight frames
//
// pedro's heading and position drift, which shows up as a constant error in the odometry angle. the limelight
// doesn't drift, so every new frame pulls x̂ back onto the real goal and the drift correction is kept in x̂
// while the tag is out of view
//
// the correction must only run on a NEW limelight frame, for the same reason as GoalDistanceFilter:
// Limelight.freshAngle() returns NaN when the frame isn't new, which skips it
//
// if you follower.setPose() or reset the heading in the middle of a match, call reset() or dθ sees the jump
// as the robot turning
@Configurable
@Config
public class GoalAngleFilter {
    public static double limelightWeight = 0.6; // 0..1 share of a new limelight frame. 1 = limelight only, 0 = pedro only
    public static double maxJump = 15;          // deg. limelight readings this far from the prediction are ignored (0 = off)
    public static int maxRejects = 10;          // after this many ignored readings in a row, trust the limelight and snap to it

    private double lastOdometry = 0; // last odometry angle given to the filter
    private double estimate = 0;     // x̂
    private double dx = 0;           // change in angle from pedro on the last update
    private boolean started = false;
    private int rejects = 0;         // limelight readings ignored in a row
    private int corrections = 0;     // limelight frames actually blended in, for telemetry

    // odometry: turret angle from pedro that points at the goal, measurement: same angle from a NEW
    // limelight frame (NaN if no tag or not a new frame)
    public double update(double odometry, double measurement) {
        if (!started) {
            // start from the limelight if we can see a tag, it doesn't drift
            estimate = Double.isNaN(measurement) ? odometry : measurement;
            lastOdometry = odometry;
            dx = 0;
            rejects = 0;
            corrections = Double.isNaN(measurement) ? 0 : 1;
            started = true;
            return estimate;
        }

        // predict. wrapped so going across ±180 is a small step, not a full turn
        dx = wrap(odometry - lastOdometry);
        estimate = wrap(estimate + dx);
        lastOdometry = odometry;

        correct(measurement);
        return estimate;
    }

    private void correct(double measurement) {
        if (Double.isNaN(measurement)) return; // no tag, or the limelight hasn't given us a new frame yet

        double error = wrap(measurement - estimate);

        // one bad frame (wrong tag, the other goal) shouldn't swing the turret
        if (maxJump > 0 && Math.abs(error) > maxJump) {
            rejects++;
            if (rejects < maxRejects) return;
            // the limelight keeps disagreeing, so it's pedro that's off
            estimate = measurement;
            rejects = 0;
            corrections++;
            return;
        }

        rejects = 0;
        corrections++;

        estimate = wrap(estimate + limelightWeight * error);
    }

    public void reset() {
        started = false;
        lastOdometry = 0;
        estimate = 0;
        dx = 0;
        rejects = 0;
        corrections = 0;
    }

    // any angle in degrees -> (-180, 180]
    public static double wrap(double degrees) {
        double a = degrees % 360;
        if (a > 180) a -= 360;
        if (a <= -180) a += 360;
        return a;
    }

    public boolean isStarted() {
        return started;
    }

    public double getEstimate() {
        return estimate;
    }

    // how much pedro moved the target on the last update (deg)
    public double getDx() {
        return dx;
    }

    public int getRejects() {
        return rejects;
    }

    // limelight frames blended in since the last reset, should climb at about the limelight's poll rate
    public int getCorrections() {
        return corrections;
    }
}
