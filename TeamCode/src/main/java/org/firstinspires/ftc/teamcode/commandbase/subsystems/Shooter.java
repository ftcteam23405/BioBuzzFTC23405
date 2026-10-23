package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.controllers.Controller;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
@Config
public class Shooter {


    private final DcMotorEx shooterMotor;

    public static double teleopPower = 0.425;
    public static double autoPower = 0.425;
    Controller controller;
    public static double kP = 0.0345;
    public static double kI = 0;
    public static double kD = 0.02;
    double previousVelocity = 0;
    double previousTime = 0;
    public static double target = 160;
    private VoltageSensor voltageSensor;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean enabled = false;
    private double lastOutput = 0;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller = Controller.pid(kP,kI,kD);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        stop();
    }

    private double limit(double n) {
        if (n>1)
            return 1;
        else if (n<-1)
            return -1;
        else
            return n;
    }

    public void stop() {
        enabled = false;
        setPower(0);
    }

    public void enable() {
        enabled = true;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public double getError() {
        return target - getVelocity();
    }

    public double getOutput() {
        return lastOutput;
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

    public CommandBuilder on() {
        return Commands.instant(this::enable);
    }

    public void start() {
        timer.reset();
        previousTime = 0;
        previousVelocity = getVelocity();
    }

    public void periodic(){
        double currentVelocity = shooterMotor.getVelocity(AngleUnit.DEGREES);
        double currentTime = timer.seconds();

        double derivative = ((currentVelocity-previousVelocity)/(currentTime-previousTime))/1000.0;
        double output = controller.calculate(target,target-currentVelocity,derivative);
        output = output * (12/voltageSensor.getVoltage());
        lastOutput = limit(output);

        //keep the previous values updating even when off so the derivative is right when it turns on
        previousVelocity = currentVelocity;
        previousTime = currentTime;

        if (enabled)
            shooterMotor.setPower(lastOutput);
    }


    public double getCurrent() {
        return shooterMotor.getCurrent(CurrentUnit.MILLIAMPS);
    }

}
