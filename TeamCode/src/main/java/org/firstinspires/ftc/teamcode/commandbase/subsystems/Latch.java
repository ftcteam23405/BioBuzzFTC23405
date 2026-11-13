package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@Config
public class Latch {
    private Servo latchServo;
    public static double closePos = 0.6;

    public static double openPos = 0.4;
    private boolean isClosed = true;

    public Latch(HardwareMap hardwareMap){
        latchServo = hardwareMap.get(Servo.class, "latch");

    }

    public void close() {
        latchServo.setPosition(closePos);
        isClosed = true;
    }

    public void open() {
        latchServo.setPosition(openPos);
        isClosed = false;
    }
    public boolean isClosed() {
        return isClosed;
    }
    public void toggle() {
        if(this.isClosed) {
            open();
        }
        else {
            close();
        }

    }


    public CommandBuilder toOpenPos() {
        return Commands.instant(this::open);
    }
    public CommandBuilder toClosePos() {
        return Commands.instant(this::close);
    }

    public boolean isOpen() {
        return latchServo.getPosition() == openPos;
    }
}