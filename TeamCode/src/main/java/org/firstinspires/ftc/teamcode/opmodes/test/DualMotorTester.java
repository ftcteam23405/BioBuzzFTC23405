package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DualMotorTester extends LinearOpMode {

    public static double speed;
    public void runOpMode() throws InterruptedException {
        final DcMotorEx motor, motor2;
        final Servo rightArmServo, leftArmServo, latchServo;
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while(opModeIsActive()){
            speed = -gamepad1.left_stick_y;

            motor.setPower(speed);
            motor2.setPower(speed);

            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addData("Motor 2 Power", motor2.getPower());
            telemetry.update();
        }
    }
}
