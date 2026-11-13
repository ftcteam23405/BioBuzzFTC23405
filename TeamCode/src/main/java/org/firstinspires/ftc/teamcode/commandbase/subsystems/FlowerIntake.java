package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlowerIntake {

    public final Servo rightFlowerServo, leftFlowerServo;

    public static double upPos = 0.66;

    public static double middlePos = 0.5;
    public static double downPos = 0.25;

    private boolean isDown = false;

    public FlowerIntake(HardwareMap hardwareMap) {
        rightFlowerServo = hardwareMap.get(Servo.class, "rightFlowerServo");
        leftFlowerServo = hardwareMap.get(Servo.class, "leftFlowerServo");
    }

    public void setPos(double pos) {
        rightFlowerServo.setPosition(1 - pos);
        leftFlowerServo.setPosition(pos);
    }

    public CommandBuilder goUp() {
        return Commands.instant(() -> setPos(upPos));
    }

    public CommandBuilder goDown() {
        return Commands.instant(() -> setPos(downPos));
    }

    public void down() {
        setPos(downPos);
        isDown = true;
    }

    public void up() {
        setPos(upPos);
        isDown = false;
    }

    public void toggle() {
        if (isDown) {
            up();
        }
        else {
            down();
        }
    }

    public boolean isDown() {
        return (isDown);
    }
}
