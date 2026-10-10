package org.firstinspires.ftc.teamcode.commandbase.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

// complementary filter for the distance from the robot to the goal center
//
// every loop:
//   1. predict: dx = (distance from the NEW pedro pose to the goal) - (distance from the OLD pedro pose to the goal)
//               x̂ = x̂ + dx, then the new pose is stored as the old pose for next time
//   2. correct: only when the limelight has a NEW frame (y),
//               x̂ = (1 - limelightWeight) * x̂ + limelightWeight * y
//
// pedro is smooth and has no lag but drifts, the limelight doesn't drift but is noisy and drops out,
// so pedro carries the estimate between frames and the limelight is what the estimate actually rides on
// (limelightWeight is 0.8 by default, so a new tag frame is 80% of the answer)
//
// the correction must only run on a NEW limelight frame. the limelight polls at 20hz and the loop runs at
// 50hz+, so blending the same frame every loop would drive x̂ onto that one reading and throw out pedro's
// motion between frames. Limelight.freshDistanceToGoal() returns NaN when the frame isn't new, which skips it
//
// the old pose is kept even when update() isn't called for a while, so dx over the gap is still right
// if you follower.setPose() in the middle of a match, call reset() or dx will see the jump as motion
@Configurable
@Config
public class GoalDistanceFilter {
    public static double limelightWeight = 0.8; // 0..1 share of a new limelight frame. 1 = limelight only, 0 = pedro only
    public static double maxJump = 12;          // inches. limelight readings this far from the prediction are ignored (0 = off)
    public static int maxRejects = 10;          // after this many ignored readings in a row, trust the limelight and snap to it

    private Pose lastPose = null;  // last pedro pose given to the filter
    private double estimate = 0;   // x̂
    private double dx = 0;         // change in distance from pedro on the last update
    private boolean started = false;
    private int rejects = 0;       // limelight readings ignored in a row
    private int corrections = 0;   // limelight frames actually blended in, for telemetry

    // robot: pedro pose, goal: goal center, measurement: NEW limelight distance (NaN if no tag or not a new frame)
    public double update(Pose robot, Pose goal, double measurement) {
        if (!started) {
            start(robot.distance(goal), measurement);
            lastPose = robot;
            return estimate;
        }

        // predict. both distances use the same goal, so switching to the other goal doesn't count as motion
        // (lastPose is null if the filter was started by update(measurement), then there's no motion to add yet)
        dx = lastPose == null ? 0 : robot.distance(goal) - lastPose.distance(goal);
        estimate += dx;
        lastPose = robot;

        correct(measurement);
        return estimate;
    }

    // no pose, limelight only (just a low pass filter, for tuners that don't run the follower)
    public double update(double measurement) {
        if (!started) {
            if (Double.isNaN(measurement)) return estimate;
            start(measurement, measurement);
            return estimate;
        }

        dx = 0;
        correct(measurement);
        return estimate;
    }

    private void start(double odometry, double measurement) {
        // start from the limelight if we can see a tag, it doesn't drift
        estimate = Double.isNaN(measurement) ? odometry : measurement;
        dx = 0;
        rejects = 0;
        corrections = Double.isNaN(measurement) ? 0 : 1;
        started = true;
    }

    private void correct(double measurement) {
        if (Double.isNaN(measurement)) return; // no tag, or the limelight hasn't given us a new frame yet

        // one bad frame (wrong tag, reflection) shouldn't yank the estimate
        if (maxJump > 0 && Math.abs(measurement - estimate) > maxJump) {
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

        estimate = (1 - limelightWeight) * estimate + limelightWeight * measurement;
    }

    public void reset() {
        started = false;
        lastPose = null;
        estimate = 0;
        dx = 0;
        rejects = 0;
        corrections = 0;
    }

    public boolean isStarted() {
        return started;
    }

    public double getEstimate() {
        return estimate;
    }

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
