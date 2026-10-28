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

// Steps the turret between two absolute positions (ticks) and scores each step. Gains live under Turret in
// Panels/Dashboard and apply next loop. Graph Target and Position. ~56.9 ticks per turret deg at 5:1.
// a = start/stop stepping   b = motor off   x = hold targetA   y = hold targetB
// init: back = resync (point the shooter forward first)
//
// HOW TO TUNE (run TurretTester first so the encoder is set up):
// 1. Set kD, kS, kV to 0. Raise kP until it reaches the target fast with a little overshoot.
// 2. Raise kD until the overshoot is gone. Too much kD makes it buzz near the target.
// 3. Set stepSize to 228 (~4 deg). Raise kS until small steps just reach the target. Too much and it hunts.
// 4. Set stepSize to 6827 (~120 deg) and check it doesn't slam. Lower maxPower if it does.
// Goal: SETTLE < 0.3 s on a 2560 tick (45 deg) step, OVERSHOOT < ~100 ticks, STEADY ERROR inside tolerance.
// Copy the final gains into Turret.java.
@Configurable
@Config
@TeleOp
public class TurretPIDTuner extends OpMode {
    public static double center = 0;       // ticks
    public static double stepSize = 2560;  // ticks, ~45 deg
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
        joinedTelemetry.addData("Gains", String.format(Locale.US, "kP %.6f  kD %.7f  kS %.3f  max %.2f",
                Turret.kP, Turret.kD, Turret.kS, Turret.maxPower));
        joinedTelemetry.addLine();
        joinedTelemetry.addLine("--- last step ---");
        joinedTelemetry.addData("SETTLE", lastSettle < 0 ? "never got inside tolerance" : String.format(Locale.US, "%.2f s", lastSettle));
        joinedTelemetry.addData("OVERSHOOT", String.format(Locale.US, "%.0f ticks  (%.2f deg)", lastOvershoot, Turret.ticksToDegrees(lastOvershoot)));
        joinedTelemetry.addData("STEADY ERROR", String.format(Locale.US, "%.0f ticks  (tolerance %.0f)", lastSteadyError, Turret.tolerance));
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
