package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;

// Steps the turret between two positions (motor encoder ticks) and scores each step. Gains live under Turret in
// Panels/Dashboard and apply next loop. Graph Target and Position. ~2.02 ticks per turret deg (725.5 per turn).
// a = start/stop stepping   b = motor off   x = hold targetA   y = hold targetB
// init: back = resync from the abs encoder (point the shooter forward first)
//
// HOW TO TUNE (run TurretTester first so both encoders are set up):
// The defaults (kP 0.02, kD 0.0016, kS 0.08) came from a sim, so first just press a and check them against
// the goal below. Only retune if they miss it.
// 1. Set kS, kV to 0 and kD to 0.001 (NOT 0, the turret is heavy and with no kD every kP overshoots ~25 deg
//    and the gear slop makes it oscillate). Raise kP until it reaches the target fast with a little overshoot.
// 2. Raise kD until the overshoot is gone. Too much kD makes it buzz near the target.
// 3. Set stepSize to 8 (~4 deg). Raise kS until small steps just reach the target. Too much and it hunts.
//    Go back to stepSize 91 and repeat 1-3 once, the gains affect each other.
// kV is tuned in TurretTrackingTester (spin with y), the tuner never moves the target so it does nothing here.
// 4. Set stepSize to 242 (~120 deg) and check it doesn't slam. Steps over wrapError (~90 deg) use wrapPower,
//    not maxPower, so lower wrapPower if it does.
// Goal: SETTLE < 0.3 s on a 91 tick (45 deg) step, OVERSHOOT < ~4 ticks, STEADY ERROR inside tolerance.
// Copy the final gains into Turret.java.
@Configurable
@Config
@TeleOp
public class TurretPIDTuner extends OpMode {
    public static double center = -362;    // ticks, middle of the 0 to -360 deg range so both steps are reachable
    public static double stepSize = 91;    // ticks, ~45 deg
    public static double holdTime = 1.5;  // s

    private Turret turret;
    private JoinedTelemetry joinedTelemetry;

    private boolean stepping = false;
    private boolean atA = true;
    private final ElapsedTime stepTimer = new ElapsedTime();

    private double stepStart = 0, stepTarget = 0;
    private double maxOvershoot = 0;
    private double settleTime = -1;
    private double lastSettle = -1, lastOvershoot = 0, lastSteadyError = 0;

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        turret = new Turret(hardwareMap);
        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
        );
    }

    @Override
    public void init_loop() {
        if (gamepad1.backWasPressed()) turret.resync();

        turret.periodic();
        joinedTelemetry.addData("Position", turret.getPosition());
        joinedTelemetry.addLine("back to resync if the shooter faces forward and Position isn't ~0");
        joinedTelemetry.addLine("a to start stepping between " + targetA() + " and " + targetB() + " ticks");
        joinedTelemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            stepping = !stepping;
            if (stepping) step(true);
        }
        if (gamepad1.bWasPressed()) {
            stepping = false;
            turret.stop();
        }
        if (gamepad1.xWasPressed()) {
            stepping = false;
            step(true);
        }
        if (gamepad1.yWasPressed()) {
            stepping = false;
            step(false);
        }

        if (stepping && stepTimer.seconds() > holdTime) step(!atA);

        turret.periodic();
        score();

        joinedTelemetry.addData("Target", turret.getTarget());
        joinedTelemetry.addData("Position", turret.getPosition());
        joinedTelemetry.addData("Error", turret.getError());
        joinedTelemetry.addData("Power", turret.getPower());
        joinedTelemetry.addData("Velocity", turret.getVelocity());
        joinedTelemetry.addData("Angle (deg)", turret.getAngle());
        joinedTelemetry.addLine();

        joinedTelemetry.addData("Mode", !turret.isEnabled() ? "OFF" : stepping ? "STEPPING (a to stop)" : "HOLDING");
        joinedTelemetry.addData("Gains", String.format(Locale.US, "kP %.4f  kD %.5f  kS %.3f  max %.2f  wrap %.2f",
                Turret.kP, Turret.kD, Turret.kS, Turret.maxPower, Turret.wrapPower));
        joinedTelemetry.addLine();
        joinedTelemetry.addLine("--- last step ---");
        joinedTelemetry.addData("SETTLE", lastSettle < 0 ? "never got inside tolerance" : String.format(Locale.US, "%.2f s", lastSettle));
        joinedTelemetry.addData("OVERSHOOT", String.format(Locale.US, "%.1f ticks  (%.2f deg)", lastOvershoot, Turret.ticksToDegrees(lastOvershoot)));
        joinedTelemetry.addData("STEADY ERROR", String.format(Locale.US, "%.1f ticks  (tolerance %.1f)", lastSteadyError, Turret.tolerance));
        joinedTelemetry.update();
    }

    private void step(boolean toA) {
        if (turret.isEnabled()) {
            lastSettle = settleTime;
            lastOvershoot = maxOvershoot;
            lastSteadyError = turret.getError();
        }

        atA = toA;
        stepStart = turret.getPosition();
        stepTarget = toA ? targetA() : targetB();
        maxOvershoot = 0;
        settleTime = -1;
        stepTimer.reset();
        turret.setTarget(stepTarget);
    }

    private void score() {
        if (!turret.isEnabled()) return;

        double direction = Math.signum(stepTarget - stepStart);
        double past = (turret.getPosition() - stepTarget) * direction;
        maxOvershoot = Math.max(maxOvershoot, past);

        boolean inside = Math.abs(turret.getError()) < Turret.tolerance;
        if (inside && settleTime < 0) settleTime = stepTimer.seconds();
        if (!inside) settleTime = -1;
    }

    private double targetA() {
        return center + stepSize / 2;
    }

    private double targetB() {
        return center - stepSize / 2;
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
