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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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



    public void stop() {
        setPower(0);
    }



    public double getVelocity() {
        return shooterMotor.getVelocity();
    }



    public void setPower(double power) {
        shooterMotor.setPower(power);
    }


    public double getCurrent() {
        return shooterMotor.getCurrent(CurrentUnit.MILLIAMPS) / 1000;
    }

}
