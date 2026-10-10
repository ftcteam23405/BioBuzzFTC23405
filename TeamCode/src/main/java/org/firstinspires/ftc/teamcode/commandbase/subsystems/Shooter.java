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




    // straight-line distance to the closest goal (inches) -> flywheel velocity (ticks/s)
    // placeholder values, fill these in with ShooterTuner. distances must be increasing

    private final DcMotorEx shooterMotor;


    // pedro pose + limelight -> filtered straight-line distance from the robot to the closest goal center

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        stop();
    }

    // spin at a fixed velocity, stops distance tracking


    // velocity follows the distance to the goal until stop() or setTargetVelocity()
    // the filter isn't reset here: it still has the last pedro pose, so it picks up where it left off
    // and keeps the drift correction it learned from the limelight

    public void stop() {
        setPower(0);
    }



    public double getVelocity() {
        return shooterMotor.getVelocity();
    }



    public void setPower(double power) {
        shooterMotor.setPower(power);
    }


    // sets the target velocity from how far we are from the goal, using odometry + limelight
    // robot: follower pose
    // limelightDistance: from Limelight.freshDistanceToGoal(), which is NaN unless the limelight gave us a
    // frame we haven't used yet. don't pass distanceToGoal() here, that reading repeats between frames and
    // the filter would blend the same number in over and over


    // sets the target velocity from a limelight distance only (no pedro), still filtered
}
