package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.commandbase.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
@Configurable
@TeleOp
public class ShooterPID extends OpMode {
    Controller controller;
    public static double kP = 0.36;
    public static double kI = 0;
    public static double kD = 0.001;
    double previousVelocity = 0;
    double previousTime = 0;
    public static double target = 145;
    private VoltageSensor voltageSensor;
    Robot robot;
    DcMotorEx motor;
    @Override
    public void init() {
        controller = Controller.pid(kP,kI,kD);
        motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        robot = new Robot(hardwareMap, Alliance.RED);
    }

    @Override
    public void loop() {

        double currentVelocity = motor.getVelocity(AngleUnit.DEGREES);
        double currentTime = getRuntime();

        double derivative = ((currentVelocity-previousVelocity)/(currentTime-previousTime))/1000.0;
        double output = controller.calculate(target,target-currentVelocity,derivative);
        output = output * (12/voltageSensor.getVoltage());

        if (gamepad1.right_bumper) {
            motor.setPower(limit(output));
        }
        else {
            motor.setPower(0);
        }
        if (gamepad1.right_trigger > 0.5) {
            robot.intake.intakeOn();
        }
        else if (gamepad1.y)
            robot.intake.intakeReverse();

        else{
            robot.intake.intakeOff();
        }
        if (gamepad1.left_trigger_pressed) {
            robot.transfer.outtake();
        }
        else if (gamepad1.x)
            robot.transfer.intake();
        else {
            robot.transfer.stop();
        }
        telemetry.addData("currentVelocity",currentVelocity);
        telemetry.addData("loop time",currentTime-previousTime);
        telemetry.addData("derivative",derivative);
        telemetry.addData("error",target-currentVelocity);
        telemetry.addData("pid",output);
        telemetry.update();

        previousVelocity = currentVelocity;
        previousTime = currentTime;
    }
    private double limit(double n) {
        if (n>1)
            return 1;
        else if (n<-1)
            return -1;
        else
            return n;
    }

}
