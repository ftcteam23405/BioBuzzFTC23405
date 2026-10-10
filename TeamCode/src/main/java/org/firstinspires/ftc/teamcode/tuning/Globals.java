package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Globals {

    public static double GATE_OPEN_POS = 0.14;
    public static double GATE_CLOSE_POS = 0.47;

    public static double shooterP = 0.003;
    public static double shooterI = 0; //use integrator (high kI) for high error response
    public static double shooterD = 0;
    public static double shooterFF = 0.0001818;

    public static double calculateTicksPerSecond(double targetRPM, double ticksPerRev) {
        return (targetRPM / 60) * ticksPerRev;
    }

    public static double calculateRPM(double tps, double ticksPerRev) {
        return (tps / ticksPerRev) * 60;
    }

}
