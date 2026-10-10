package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlowerIntake {

    public final Servo rightFlowerServo, leftFlowerServo;

    public static double upPos = 0.5;
    public static double downPos = 0.5;

    public FlowerIntake(HardwareMap hardwareMap) {
        rightFlowerServo = hardwareMap.get(Servo.class, "rightFlowerServo");
        leftFlowerServo = hardwareMap.get(Servo.class, "leftFlowerServo");
    }

    public void setPos(double pos) {
        rightFlowerServo.setPosition(1 - pos);
        leftFlowerServo.setPosition(pos);
    }

    public CommandBuilder up() {
        return Commands.instant(() -> setPos(upPos));
    }

    public CommandBuilder down() {
        return Commands.instant(() -> setPos(downPos));
    }

    public boolean isDown() {
        return ((leftFlowerServo.getPosition() - downPos) == 0);
    }
}
