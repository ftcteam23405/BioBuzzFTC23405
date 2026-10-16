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

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;



// tunes the flywheel velocity controller in Shooter (kS, kV, kP). gains are live under Shooter in panels / dashboard
// graph Target and Velocity to see the step response

// a = start/stop stepping   b = spin down (coast)   x = hold lowVelocity   y = hold highVelocity
// dpad up = feedforward sweep: runs fixed powers with no PID and fits kS + kV to what the flywheel actually does

// how to tune, in order:
// 1. dpad up and wait for the sweep to finish, copy the kS and kV it prints into Shooter. set kP to 0 and press a,
//    the flywheel should already land close to every target on feedforward alone (STEADY ERROR small)
// 2. raise kP until it gets to the target quickly and STEADY ERROR is ~0. too much and Velocity / Power oscillate
//    once it's there. it can't brake (power is clipped at 0), so stepping down is always a slow coast, that's fine
// 3. hold a velocity (x / y) and feed a few balls through. DIP is how far each shot pulls it down, RECOVERY is how
//    long it takes to get back inside Shooter.tolerance. more kP = faster recovery, as long as it doesn't oscillate
// good is SETTLE under ~1 s for a step up, small OVERSHOOT, STEADY ERROR inside tolerance, RECOVERY under ~0.3 s
@Configurable
@Config
@TeleOp
public class ShooterPIDTuner extends OpMode {
    public static double lowVelocity = 1100;   // ticks/s, steps go between these two
    public static double highVelocity = 1400;
    public static double holdTime = 3;         // seconds at each target before stepping again
    public static double settleConfirm = 0.25; // seconds it has to stay inside tolerance to count as settled

    // feedforward sweep
    public static double sweepStartPower = 0.2;
    public static double sweepEndPower = 0.9;
    public static int sweepSteps = 8;
    public static double sweepSettleTime = 2.5;  // seconds at each power before it's measured, the flywheel is slow
    public static double sweepAverageTime = 0.5; // seconds of velocity averaged at the end of each power

    private Shooter shooter;
    private JoinedTelemetry joinedTelemetry;

    private boolean stepping = false;
    private boolean atHigh = false;
    private final ElapsedTime stepTimer = new ElapsedTime();

    // scored on every step
    private double stepStart = 0, stepTarget = 0;
    private double maxOvershoot = 0;
    private double settleTime = -1;    // first time it got inside tolerance and stayed there, -1 = not yet
    private boolean settled = false;   // stayed inside for settleConfirm, from here on drops are shots
    private double steadySum = 0;
    private int steadyCount = 0;
    private double lastSettle = -1, lastOvershoot = 0, lastSteadyError = Double.NaN;

    // shot recovery, only once it has settled
    private boolean inDip = false;
    private double dipStart = 0, dipMin = 0;
    private double lastDip = Double.NaN, lastRecovery = Double.NaN;

    // feedforward sweep
    private boolean sweeping = false;
    private int sweepIndex = 0;
    private final ElapsedTime sweepTimer = new ElapsedTime();
    private double sweepSum = 0;
    private int sweepCount = 0;
    private final List<double[]> sweepPoints = new ArrayList<>(); // {velocity, power}
    private double fitKS = Double.NaN, fitKV = Double.NaN;

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        shooter = new Shooter(hardwareMap);
        joinedTelemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
        );
    }

    @Override
    public void init_loop() {
        joinedTelemetry.addLine("a to start stepping between " + lowVelocity + " and " + highVelocity + " ticks/s");
        joinedTelemetry.addLine("dpad up to run the feedforward sweep first if kS / kV aren't tuned yet");
        joinedTelemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            sweeping = false;
            stepping = !stepping;
            if (stepping) step(true);
        }
        if (gamepad1.bWasPressed()) {
            sweeping = false;
            stepping = false;
            shooter.stop();
        }
        if (gamepad1.xWasPressed()) {
            sweeping = false;
            stepping = false;
            step(false);
        }
        if (gamepad1.yWasPressed()) {
            sweeping = false;
            stepping = false;
            step(true);
        }
        if (gamepad1.dpadUpWasPressed()) startSweep();

        if (stepping && stepTimer.seconds() > holdTime) step(!atHigh);
        if (sweeping) updateSweep();

        shooter.periodic();
        if (!sweeping) score();

        // numbers first so they graph
        joinedTelemetry.addData("Target", shooter.getTargetVelocity());
        joinedTelemetry.addData("Velocity", shooter.getVelocity());
        joinedTelemetry.addData("Error", shooter.getTargetVelocity() - shooter.getVelocity());
        joinedTelemetry.addData("Power", shooter.getPower());
        joinedTelemetry.addLine();

        joinedTelemetry.addData("Mode", sweeping ? "SWEEP (b to cancel)"
                : shooter.getTargetVelocity() == 0 ? "OFF"
                : stepping ? "STEPPING (a to stop)" : "HOLDING");
        joinedTelemetry.addData("Gains", String.format(Locale.US, "kS %.3f  kV %.6f  kP %.4f  tolerance %.0f",
                Shooter.kS, Shooter.kV, Shooter.kP, Shooter.tolerance));
        joinedTelemetry.addLine();

        if (sweeping || !sweepPoints.isEmpty()) addSweepTelemetry();
        else addStepTelemetry();
        joinedTelemetry.update();
    }

    private void addStepTelemetry() {
        joinedTelemetry.addLine("--- last step ---");
        joinedTelemetry.addData("SETTLE", lastSettle < 0 ? "never settled inside tolerance" : String.format(Locale.US, "%.2f s", lastSettle));
        joinedTelemetry.addData("OVERSHOOT", String.format(Locale.US, "%.0f ticks/s", lastOvershoot));
        joinedTelemetry.addData("STEADY ERROR", Double.isNaN(lastSteadyError) ? "-" : String.format(Locale.US, "%.0f ticks/s  (tolerance %.0f)", lastSteadyError, Shooter.tolerance));
        joinedTelemetry.addLine("--- last shot ---");
        joinedTelemetry.addData("DIP", Double.isNaN(lastDip) ? "-" : String.format(Locale.US, "%.0f ticks/s", lastDip));
        joinedTelemetry.addData("RECOVERY", Double.isNaN(lastRecovery) ? "-" : String.format(Locale.US, "%.2f s", lastRecovery));
    }

    private void addSweepTelemetry() {
        joinedTelemetry.addLine("--- feedforward sweep (power -> ticks/s) ---");
        if (sweeping)
            joinedTelemetry.addData("Running", String.format(Locale.US, "power %.2f  (%d / %d)", sweepPower(sweepIndex), sweepIndex + 1, sweepSteps));
        for (double[] p : sweepPoints)
            joinedTelemetry.addLine(String.format(Locale.US, "%.2f -> %.0f", p[1], p[0]));
        if (!Double.isNaN(fitKS)) {
            joinedTelemetry.addLine();
            joinedTelemetry.addLine(String.format(Locale.US, "kS = %.3f", fitKS));
            joinedTelemetry.addLine(String.format(Locale.US, "kV = %.6f", fitKV));
            joinedTelemetry.addLine("copy these into Shooter, then press a to check the steps");
        } else if (!sweeping) {
            joinedTelemetry.addLine("not enough points to fit, widen the sweep powers");
        }
    }

    private void step(boolean toHigh) {
        // finish scoring the step we're leaving
        if (shooter.getTargetVelocity() > 0) {
            lastSettle = settled ? settleTime : -1;
            lastOvershoot = maxOvershoot;
            lastSteadyError = steadyCount > 0 ? steadySum / steadyCount : Double.NaN;
        }

        atHigh = toHigh;
        stepStart = shooter.getVelocity();
        stepTarget = toHigh ? highVelocity : lowVelocity;
        maxOvershoot = 0;
        settleTime = -1;
        settled = false;
        steadySum = 0;
        steadyCount = 0;
        inDip = false;
        stepTimer.reset();
        shooter.setTargetVelocity(stepTarget);
        sweepPoints.clear(); // back to showing the step scores
    }

    private void score() {
        if (shooter.getTargetVelocity() == 0) return;

        double t = stepTimer.seconds();
        double velocity = shooter.getVelocity();
        double error = stepTarget - velocity;
        boolean inside = Math.abs(error) < Shooter.tolerance;

        if (!settled) {
            // overshoot = how far it went past the target, in the direction it was going
            double direction = Math.signum(stepTarget - stepStart);
            maxOvershoot = Math.max(maxOvershoot, -error * direction);

            if (!inside) settleTime = -1; // left the band again, it hasn't settled yet
            else if (settleTime < 0) settleTime = t;
            else if (t - settleTime > settleConfirm) settled = true;
            return;
        }

        // settled: anything that drops it below the band is a shot going through
        if (!inDip && velocity < stepTarget - Shooter.tolerance) {
            inDip = true;
            dipStart = t;
            dipMin = velocity;
        }
        if (inDip) {
            dipMin = Math.min(dipMin, velocity);
            if (inside) {
                inDip = false;
                lastDip = stepTarget - dipMin;
                lastRecovery = t - dipStart;
            }
            return;
        }

        steadySum += error;
        steadyCount++;
    }

    private void startSweep() {
        stepping = false;
        sweeping = true;
        sweepIndex = 0;
        sweepPoints.clear();
        fitKS = Double.NaN;
        fitKV = Double.NaN;
        startSweepStep();
    }

    private void startSweepStep() {
        sweepSum = 0;
        sweepCount = 0;
        sweepTimer.reset();
        shooter.setOpenLoopPower(sweepPower(sweepIndex));
    }

    private void updateSweep() {
        double t = sweepTimer.seconds();
        if (t > sweepSettleTime - sweepAverageTime) {
            sweepSum += shooter.getVelocity();
            sweepCount++;
        }
        if (t < sweepSettleTime) return;

        if (sweepCount > 0) sweepPoints.add(new double[] {sweepSum / sweepCount, sweepPower(sweepIndex)});
        sweepIndex++;

        if (sweepIndex < Math.max(2, sweepSteps)) {
            startSweepStep();
        } else {
            sweeping = false;
            shooter.stop();
            fitFeedforward();
        }
    }

    private double sweepPower(int i) {
        int steps = Math.max(2, sweepSteps);
        return sweepStartPower + (sweepEndPower - sweepStartPower) * i / (steps - 1);
    }

    // least squares line through the sweep: power = kS + kV * velocity
    // points where the flywheel didn't move are left out, they're below kS and would drag the line down
    private void fitFeedforward() {
        double n = 0, sumV = 0, sumP = 0, sumVV = 0, sumVP = 0;
        for (double[] p : sweepPoints) {
            if (p[0] < 50) continue;
            n++;
            sumV += p[0];
            sumP += p[1];
            sumVV += p[0] * p[0];
            sumVP += p[0] * p[1];
        }

        double denominator = n * sumVV - sumV * sumV;
        if (n < 2 || Math.abs(denominator) < 1e-9) return;

        fitKV = (n * sumVP - sumV * sumP) / denominator;
        fitKS = (sumP - fitKV * sumV) / n;
    }

    @Override
    public void stop() {
        shooter.stop();
    }
}
