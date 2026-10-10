package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class ServoTester extends LinearOpMode {

    public static double flowerPos = 0.5, latchPos = 0.5, llPos = 0.5;

    public void runOpMode() throws InterruptedException {
        final Servo rightFlowerServo, leftFlowerServo, latchServo, llServo;
        rightFlowerServo = hardwareMap.get(Servo.class, "rightFlowerServo");
        leftFlowerServo = hardwareMap.get(Servo.class, "leftFlowerServo");
        latchServo = hardwareMap.get(Servo.class, "latchServo");
        llServo = hardwareMap.get(Servo.class, "llServo");
        waitForStart();


        while(opModeIsActive()){
            latchServo.setPosition(latchPos);
            rightFlowerServo.setPosition(1 - flowerPos);
            leftFlowerServo.setPosition(flowerPos);
            llServo.setPosition(llPos);
        }
    }

}
