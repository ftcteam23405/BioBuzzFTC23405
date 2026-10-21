package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
@Config
public class Latch {
    private final CRServo servo;
    public static double closePos = 0.52;
    public static double intakePos = 0.3;
    public static double openPos = 0.2;

    public Latch(HardwareMap hardwareMap){
        servo = hardwareMap.get(CRServo.class, "latchServo");
    }

    public void intake(){servo.setPower(1);}
    public void outtake(){servo.setPower(-1);}
    public void stop(){servo.setPower(0);}



    public CommandBuilder doIntake() {
        return Commands.instant(this::intake);
    }
    public CommandBuilder doOutake() { return Commands.instant(this::outtake); }

    public CommandBuilder off() {
        return Commands.instant(this::stop);
    }
}
