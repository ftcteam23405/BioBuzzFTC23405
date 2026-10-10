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
    public static double closePos = 0.52;
    public static double intakePos = 0.3;
    public static double openPos = 0.2;

    public Latch(HardwareMap hardwareMap){
        latchServo = hardwareMap.get(Servo.class, "latchServo");
    }

    public void close() {
        latchServo.setPosition(closePos);
    }

    public void open() {
        latchServo.setPosition(openPos);
    }

    public void intake() {
        latchServo.setPosition(intakePos);
    }

    public CommandBuilder toIntakePos() {
        return Commands.instant(this::intake);
    }
    public CommandBuilder toOpenPos() { return Commands.instant(this::open); }
    public CommandBuilder toClosePos() { return Commands.instant(this::close); }

    public boolean isOpen() {
        return latchServo.getPosition() == openPos;
    }
}
