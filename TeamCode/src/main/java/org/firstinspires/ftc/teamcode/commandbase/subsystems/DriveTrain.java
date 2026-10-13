package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DriveTrain {
    public final DcMotorEx fl;
    public final DcMotorEx fr;
    public final DcMotorEx bl;
    public final DcMotorEx br;

    public DriveTrain (HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
    }

    public double getCurrent() {
        return fl.getCurrent(CurrentUnit.MILLIAMPS)+ fr.getCurrent(CurrentUnit.MILLIAMPS)+bl.getCurrent(CurrentUnit.MILLIAMPS)+br.getCurrent(CurrentUnit.MILLIAMPS);
    }

}
