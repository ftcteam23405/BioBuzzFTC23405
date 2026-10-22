package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
@Config
public class Shooter {


    private final DcMotorEx shooterMotor;

    public static double teleopPower = 0.425;
    public static double autoPower = 0.425;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        stop();
    }



    public void stop() {
        setPower(0);
    }



    public double getVelocity() {
        return shooterMotor.getVelocity(AngleUnit.DEGREES);
    }

    public void setTeleopPresetPower() {
        shooterMotor.setPower(teleopPower);
    }

    public void setAutoPresetPower() {
        shooterMotor.setPower(autoPower);
    }

    public void setPower(double power) {
        shooterMotor.setPower(power);
    }

    public CommandBuilder off() {
        return Commands.instant(this::stop);
    }

    public CommandBuilder autoPower() {
        return Commands.instant(this::setAutoPresetPower);
    }



    public double getCurrent() {
        return shooterMotor.getCurrent(CurrentUnit.MILLIAMPS);
    }

}
