package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Light;

@TeleOp
public class tele extends OpMode {
    DcMotorEx shooterMotor;
    DcMotorEx turretMotor;
    DcMotorEx intakeMotor;
    Servo latchServo;
    Servo leftFlowerMech,rightFlowerMech;
    Light light;
    double latchpos = 0.5;
    double flowerpos = 0.5;
    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        latchServo = hardwareMap.get(Servo.class, "latchServo");
        leftFlowerMech = hardwareMap.get(Servo.class, "leftFlowerServo");
        rightFlowerMech = hardwareMap.get(Servo.class, "rightFlowerServo");
        light = new Light(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            turretMotor.setPower(0.25);
        }
        else if (gamepad1.left_bumper) {
            turretMotor.setPower(-0.25);
        }
        else {
            turretMotor.setPower(0);
        }

        if (gamepad1.left_trigger_pressed) {
            intakeMotor.setPower(1);
            light.toIntakeFullColor();
        }
        else if (gamepad1.right_trigger_pressed) {
            intakeMotor.setPower(-1);
            light.intakeNotFull();
        }
        else {
            intakeMotor.setPower(0);
        }
        if (gamepad1.aWasPressed()) {
            flowerpos += 0.01;
        }
        else if (gamepad1.bWasPressed()) {
            flowerpos -= 0.01;
        }
        if (gamepad1.xWasPressed()) {
            latchpos += 0.01;
        }
        else if (gamepad1.yWasPressed()) {
            latchpos -= 0.01;
        }
        latchServo.setPosition(latchpos);
        //leftFlowerMech.setPosition(flowerpos);
        //rightFlowerMech.setPosition(1-flowerpos);


        shooterMotor.setPower(-flowerpos);
        telemetry.addData("latch pos", latchpos);
        telemetry.addData("flower pos", flowerpos);
        telemetry.update();
    }
}
