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



// under Turret and they take effect on the next loop. graph Target and Angle to see the step response

// a = start/stop stepping   b = motor off   x = hold targetA   y = hold targetB

// how to tune, in order (run TurretTester first so the encoder and directions are right):
// 1. everything 0 but kP. raise kP until it gets to the target quickly and overshoots a little
// 2. raise kD until the overshoot is gone. too much kD and it buzzes / chatters near the target
// 3. kS: if it stops short of small steps (try stepSize 3), raise kS until it just makes it. too much and it
//    hunts back and forth around the target
// 4. kI only if there's still steady state error after kS, and keep it small. iZone keeps it from winding up
// 5. try a big step (stepSize 120) to check it doesn't slam into a hard stop, lower maxPower if it does
// good is SETTLE under about 0.3 s for a 45 deg step, OVERSHOOT under 1-2 deg, STEADY ERROR inside tolerance
@Configurable
@Config
@TeleOp
public class TurretPIDTuner extends OpMode {
    public static double center = 0;      // steps are center +/- stepSize / 2
    public static double stepSize = 45;
    public static double holdTime = 1.5;  // seconds at each target before stepping again

    private Turret turret;
    private JoinedTelemetry joinedTelemetry;

    private boolean stepping = false;
    private boolean atA = true;
    private final ElapsedTime stepTimer = new ElapsedTime();

    // scored on every step
    private double stepStart = 0, stepTarget = 0;
    private double maxOvershoot = 0;
    private double settleTime = -1;      // first time it got inside tolerance and stayed there, -1 = not yet
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
        turret.periodic();
        joinedTelemetry.addData("Angle", turret.getAngle());
        joinedTelemetry.addLine("a to start stepping between " + targetA() + " and " + targetB());
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

        // numbers first so they graph
        joinedTelemetry.addData("Target", turret.getTargetAngle());
        joinedTelemetry.addData("Angle", turret.getAngle());
        joinedTelemetry.addData("Error", turret.getError());
        joinedTelemetry.addData("Power", turret.getPower());
        joinedTelemetry.addData("Velocity", turret.getVelocity());
        joinedTelemetry.addLine();

        joinedTelemetry.addData("Mode", !turret.isEnabled() ? "OFF" : stepping ? "STEPPING (a to stop)" : "HOLDING");
        joinedTelemetry.addData("Gains", String.format(Locale.US, "kP %.4f  kI %.4f  kD %.5f  kS %.3f  max %.2f",
                Turret.kP, Turret.kI, Turret.kD, Turret.kS, Turret.maxPower));
        joinedTelemetry.addLine();
        joinedTelemetry.addLine("--- last step ---");
        joinedTelemetry.addData("SETTLE", lastSettle < 0 ? "never got inside tolerance" : String.format(Locale.US, "%.2f s", lastSettle));
        joinedTelemetry.addData("OVERSHOOT", String.format(Locale.US, "%.2f deg", lastOvershoot));
        joinedTelemetry.addData("STEADY ERROR", String.format(Locale.US, "%.2f deg  (tolerance %.1f)", lastSteadyError, Turret.tolerance));
        joinedTelemetry.update();
    }

    private void step(boolean toA) {
        // finish scoring the step we're leaving
        if (turret.isEnabled()) {
            lastSettle = settleTime;
            lastOvershoot = maxOvershoot;
            lastSteadyError = turret.getError();
        }

        atA = toA;
        stepStart = turret.getAngle();
        stepTarget = toA ? targetA() : targetB();
        maxOvershoot = 0;
        settleTime = -1;
        stepTimer.reset();
        turret.setTargetAngle(stepTarget);
    }

    private void score() {
        if (!turret.isEnabled()) return;

        // overshoot = how far it went past the target, in the direction it was moving
        double direction = Math.signum(stepTarget - stepStart);
        double past = (turret.getAngle() - stepTarget) * direction;
        maxOvershoot = Math.max(maxOvershoot, past);

        boolean inside = Math.abs(turret.getError()) < Turret.tolerance;
        if (inside && settleTime < 0) settleTime = stepTimer.seconds();
        if (!inside) settleTime = -1; // left the band again, it hasn't settled yet
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
