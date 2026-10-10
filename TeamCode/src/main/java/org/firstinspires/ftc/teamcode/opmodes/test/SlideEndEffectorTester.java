package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SlideEndEffectorTester extends LinearOpMode {

    public static double speed, intakeSpeed;
    public void runOpMode() throws InterruptedException {
        final DcMotorEx motor, motor2, intakeMotor;
        final Servo rightArmServo, leftArmServo, latchServo;
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        latchServo = hardwareMap.get(Servo.class, "latchServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        double latchInitPos = latchServo.getPosition();
        double rightInitPos = rightArmServo.getPosition();
        double leftInitPos = leftArmServo.getPosition();

        while(opModeIsActive()){
            speed = -gamepad1.left_stick_y;
            intakeSpeed = -gamepad1.right_stick_y;

            motor.setPower(speed);
            motor2.setPower(speed);
            intakeMotor.setPower(intakeSpeed);

            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addData("Motor 2 Power", motor2.getPower());
            telemetry.addData("Latch Position", latchServo.getPosition());
            telemetry.addData("Right Arm Position", rightArmServo.getPosition());
            telemetry.addData("Left Arm Position", leftArmServo.getPosition());
            telemetry.update();
        }
    }
}
