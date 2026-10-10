package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
@Config
public class Intake {
    private final DcMotorEx intakeMotor;
    public static double intakeOffSpeed = 0;
    public static double intakeOnSpeed = 1;
    public static double intakeReverseSpeed = -1;

    private final DigitalChannel laser;

    public Intake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        laser = hardwareMap.get(DigitalChannel.class, "laser");
        laser.setMode(DigitalChannel.Mode.INPUT);
    }

    public void setPower(double power){
        intakeMotor.setPower(power);
    }

    public void intakeOff() {
        setPower(intakeOffSpeed);
    }

    public void intakeOn() {
        setPower(intakeOnSpeed);
    }

    public void intakeReverse() {
        setPower(intakeReverseSpeed);
    }

    public boolean isFull() {
        return laser.getState();
    }

    public CommandBuilder off() {
        return Commands.instant(() -> setPower(intakeOffSpeed));
    }

    public CommandBuilder on() {
        return Commands.instant(() -> setPower(intakeOnSpeed));
    }

    public CommandBuilder reverse() {
        return Commands.instant(() -> setPower(intakeReverseSpeed));
    }

    public String getCurrent() {
        return "Intake Motor: " + intakeMotor.getCurrent(CurrentUnit.AMPS);
    }
}
