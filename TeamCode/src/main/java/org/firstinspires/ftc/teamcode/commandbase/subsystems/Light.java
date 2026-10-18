package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@Config
public class Light {

    private Servo rpmLight, intakeLight;

    public static double atShooterTargetColor = 0.722;
    public static double intakeFullColor = 0.5;
    public static double defaultColor = 0.277;

    public Light(HardwareMap hardwareMap) {
//        rpmLight = hardwareMap.get(Servo.class, "rpmLight");
        intakeLight = hardwareMap.get(Servo.class, "intakeLight");
    }

    public void toIntakeFullColor() {
        intakeLight.setPosition(intakeFullColor);
    }

//    public void toAtTargetColor() {
//        rpmLight.setPosition(atShooterTargetColor);
//    }

//    public void notAtRPM() {
//        rpmLight.setPosition(defaultColor);
//    }

    public void intakeNotFull() {
        intakeLight.setPosition(defaultColor);
    }
}
