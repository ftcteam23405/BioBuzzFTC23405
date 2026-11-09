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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Collections;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;

// automatically tunes the turret position controller in Turret (kS, kV, kP, kD)
// run TurretTester first so motorReversed / the abs encoder are right and Position reads ~0 with the shooter forward.
// clear the turret's whole swing (it runs from end to end), press a and leave it alone for ~1.5 min.
// when it's done it prints the four lines to paste into Turret.java (already applied live until restart)
//
// a = full auto tune   dpad up = feedforward only (kS, kV)   dpad right = PD + kS check only (keeps kV)
// x = hold center with the current gains   b = stop / abort (abort puts the old gains back)
// init: back = resync from the abs encoder (point the shooter forward first)
// graph Target, Position, Error, Power and Velocity in panels / dashboard to watch it work
//
// everything is in MOTOR ENCODER TICKS from forward, same as Turret (0 = forward, + = left, range minTicks..maxTicks,
// ~2.02 ticks per deg). it only ever drives the turret through Turret.setTarget / setOpenLoopPower + periodic(),
// so the real controller is what gets scored:
//   out = kP * error + kD * (targetRate - velocity) + kV * targetRate + kS * sign(error) (outside tolerance)
//   clipped to maxPower (wrapPower past wrapError), cut at the soft limits
//
// what it does:
// 1. feedforward sweep: runs fixed powers with no PID from one end of the swing to the other (both directions),
//    measures the speed it settles at and fits |power| = kS + kV * |speed|. that slope is exactly Turret.kV
//    (power per tick/s of target motion) and the intercept is the friction kS has to beat. it also measures the
//    time constant tau (how long the turret takes to get up to speed), and checks the abs encoder still agrees
// 2. PD search: with the motor modelled as speed' = (power / kV - speed) / tau, PD on position gives
//    tau s^2 + (1 + kD / kV) s + kP / kV = 0. so for a natural frequency wn and damping zeta:
//       kP = wn^2 * tau * kV       kD = (2 * zeta * wn * tau - 1) * kV   (never below minKD, slop needs some)
//    it steps stepSize back and forth with bigger and bigger wn until a step overshoots, oscillates or buzzes,
//    takes the smallest wn that settles about as fast as the fastest, then tries a bit less / more damping there
// 3. kS check: small steps (smallStep). if they stop short of tolerance it raises kS, if it hunts it lowers it
// 4. validation: normal steps plus one bigStep (over wrapError, so it uses wrapPower) and reports them, then holds
@Configurable
@Config
@TeleOp
public class TurretAutoTuner extends OpMode {
    // where it steps, ticks
    public static double center = -362;     // middle of the swing
    public static double stepSize = 91;     // ~45 deg, keep under Turret.wrapError so steps use maxPower
    public static double bigStep = 242;     // ~120 deg, validation only, checks wrapPower doesn't slam
    public static double smallStep = 8;     // ~4 deg, for the kS check
    public static double stepHold = 1.2;    // s per step
    public static double smallStepHold = 0.8;

    // feedforward sweep
    public static double sweepStartPower = 0.12;
    public static double sweepEndPower = 0.45;
    public static int sweepSteps = 6;       // powers, each one run both directions
    public static double edgeMargin = 60;   // ticks kept clear of minTicks / maxTicks
    public static double brakeLead = 0.08;  // s of travel at the current speed it brakes early by

    // PD search
    public static double zeta = 1.0;        // damping, 1 = no overshoot in the model
    public static double wnRatio = 1.3;     // each wn is this many times the last
    public static int pdTests = 12;
    public static double minKD = 0.0005;    // kD floor, with ~0 kD the gear slop makes it oscillate
    public static double defaultTimeConstant = 0.08; // s, tau used for dpad right if the sweep hasn't run
    public static double settleBand = 4;    // ticks, "settled" for the PD search (kS check uses Turret.tolerance)
    public static double overshootLimit = 4; // ticks
    public static double buzzVelocity = 40; // ticks/s velocity noise once settled that counts as buzzing
    public static double settleMargin = 0.15; // picks the smallest wn within this fraction of the fastest settle

    // kS check
    public static int kSTries = 8;

    private static final double STILL_VELOCITY = 15;  // ticks/s
    private static final double STILL_CONFIRM = 0.2;  // s
    private static final double MOVE_TIMEOUT = 3;
    private static final double SETTLE_CONFIRM = 0.15; // s it has to stay inside the band to count as settled
    private static final double FINAL_WINDOW = 0.3;    // s at the end of a step used for steady error + noise
    private static final double STEADY_WINDOW = 0.12;  // s, two windows the same speed at the brake point = steady
    private static final double STALL_TIME = 1.0;      // s without moving = power is under kS
    private static final double SWEEP_RUN_TIMEOUT = 4;
    private static final double MIN_SPEED = 20;        // ticks/s, slower sweep points are left out of the fit
    private static final double LOW_BATTERY = 12.5;

    private enum Stage { IDLE, MOVE, SWEEP_RUN, SWEEP_STOP, STEPS, HOLD }
    private enum Run { FULL, FEEDFORWARD, PD }
    private enum Purpose { PD, KS, VALIDATE }

    // one step of a step test
    private static class StepResult {
        double start, target;
        double settle = Double.NaN;   // s to inside the band for good, NaN = never
        double overshoot = 0;         // ticks past the target
        int flips = 0;                // error sign changes outside the band, 1 = overshot, 2+ = oscillating
        int lastSign = 0;
        double insideSince = -1;
        double errorSum, speedSum, speedSquareSum;
        int finalCount;
        double steadyError, noise;    // over the last FINAL_WINDOW
    }

    private static class PDTest {
        double wn, zeta, kP, kD;
        double settle = Double.NaN, overshoot, noise, steadyError;
        int flips;
        boolean ok;
        String verdict;
    }

    private static class KSTest {
        double kS;
        int reached, steps;
        boolean hunting;
    }

    private Turret turret;
    private JoinedTelemetry joinedTelemetry;

    private Stage stage = Stage.IDLE;
    private Run run = Run.FULL;
    private final ElapsedTime stageTimer = new ElapsedTime();

    private double savedKS, savedKV, savedKP, savedKD;
    private String status = "";
    private final List<String> warnings = new ArrayList<>();
    private boolean finished = false;

    // move
    private Runnable afterMove;
    private double stillSince = -1;

    // feedforward sweep
    private int sweepRun = 0;
    private double sweepSign = -1, sweepStartPos = 0;
    private final List<double[]> samples = new ArrayList<>();     // {t, speed}
    private final List<double[]> sweepPoints = new ArrayList<>(); // {speed, |power|, sign}
    private final List<String> sweepLog = new ArrayList<>();
    private final List<Double> timeConstants = new ArrayList<>();
    private double timeConstant = Double.NaN;
    private double fitKS = Double.NaN, fitKV = Double.NaN, fitError = Double.NaN;
    private boolean ranOutOfTravel = false;
    private double absCheck = Double.NaN;

    // step tests
    private Purpose purpose;
    private double[] stepTargets;
    private int stepIndex;
    private double hold, band;
    private StepResult step;
    private final List<StepResult> stepResults = new ArrayList<>();

    // PD search
    private double tau;
    private boolean refining = false;
    private final List<PDTest> pdResults = new ArrayList<>();
    private PDTest currentPD, pickedPD;
    private final List<Double> refineZetas = new ArrayList<>();

    // kS check
    private final List<KSTest> kSResults = new ArrayList<>();
    private double kSLow = 0, kSHigh = Double.POSITIVE_INFINITY; // low = stopped short, high = hunted

    // validation
    private final List<StepResult> validation = new ArrayList<>();

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
        joinedTelemetry.addData("Angle (deg)", turret.getAngle());
        joinedTelemetry.addLine("back to resync if the shooter faces forward and Position isn't ~0");
        joinedTelemetry.addLine("clear the whole swing, press start then a for the full auto tune (~1.5 min)");
        double voltage = batteryVoltage();
        joinedTelemetry.addData("Battery", String.format(Locale.US, "%.1f V%s", voltage,
                voltage < LOW_BATTERY ? "  (low, charge it first, kS / kV depend on it)" : ""));
        joinedTelemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) startRun(Run.FULL);
        if (gamepad1.dpadUpWasPressed()) startRun(Run.FEEDFORWARD);
        if (gamepad1.dpadRightWasPressed()) startRun(Run.PD);
        if (gamepad1.xWasPressed()) toggleHold();
        if (gamepad1.bWasPressed()) abort();

        // readings are from last loop's periodic(), commands set here go out in this loop's periodic()
        double t = stageTimer.seconds();
        switch (stage) {
            case MOVE: updateMove(t); break;
            case SWEEP_RUN: updateSweepRun(t); break;
            case SWEEP_STOP: updateSweepStop(t); break;
            case STEPS: updateSteps(t); break;
            default: break;
        }

        turret.periodic();
        addTelemetry();
    }

    // ---------------------------------------------------------------- runs

    private void startRun(Run r) {
        if (running()) return; // b first to abort

        String problem = checkSetup(r);
        if (problem != null) {
            status = "can't start: " + problem;
            return;
        }

        run = r;
        savedKS = Turret.kS;
        savedKV = Turret.kV;
        savedKP = Turret.kP;
        savedKD = Turret.kD;
        status = "";
        warnings.clear();
        finished = false;
        pdResults.clear();
        pickedPD = null;
        kSResults.clear();
        validation.clear();

        double voltage = batteryVoltage();
        if (voltage < LOW_BATTERY)
            warnings.add(String.format(Locale.US, "battery is %.1f V, kS / kV come out different on a charged one", voltage));
        if (stepSize >= Turret.wrapError)
            warnings.add("stepSize is over Turret.wrapError, the PD steps run at wrapPower instead of maxPower");

        if (r == Run.PD) startPDSearch();
        else startSweep();
    }

    // null = fine
    private String checkSetup(Run r) {
        double lo = Turret.minTicks, hi = Turret.maxTicks;
        double reach = Math.max(Math.max(stepSize, bigStep), 2 * smallStep) / 2;
        if (center - reach < lo || center + reach > hi)
            return String.format(Locale.US, "center %.0f +/- %.0f doesn't fit in minTicks %.0f .. maxTicks %.0f",
                    center, reach, lo, hi);
        if (r != Run.PD && hi - lo - 2 * edgeMargin < 200)
            return "less than 200 ticks of swing between the edge margins, lower edgeMargin";
        if (r == Run.PD && Turret.kV <= 0) return "Turret.kV is 0, run the feedforward sweep first (dpad up or a)";
        return null;
    }

    private boolean running() {
        return stage != Stage.IDLE && stage != Stage.HOLD;
    }

    private void enter(Stage next) {
        stage = next;
        stageTimer.reset();
        samples.clear();
        stillSince = -1;
    }

    private void abort() {
        if (running()) {
            restoreGains();
            status = "aborted, old gains put back";
        }
        turret.stop();
        enter(Stage.IDLE);
    }

    private void fail(String message) {
        restoreGains();
        turret.stop();
        status = "FAILED: " + message;
        enter(Stage.IDLE);
    }

    private void restoreGains() {
        Turret.kS = savedKS;
        Turret.kV = savedKV;
        Turret.kP = savedKP;
        Turret.kD = savedKD;
    }

    private void finish() {
        finished = true;
        status = "done! holding center with the new gains (b to stop)";
        startHold();
    }

    private void toggleHold() {
        if (running()) return;
        if (stage == Stage.HOLD) {
            turret.stop();
            enter(Stage.IDLE);
        } else {
            startHold();
        }
    }

    private void startHold() {
        turret.setTarget(center);
        enter(Stage.HOLD);
    }

    // ---------------------------------------------------------------- move (PID with whatever gains are set)

    private void moveTo(double ticks, Runnable then) {
        afterMove = then;
        turret.setTarget(ticks);
        enter(Stage.MOVE);
    }

    private void updateMove(double t) {
        if (stillFor(t, Math.abs(turret.getError()) < settleBand) < STILL_CONFIRM && t < MOVE_TIMEOUT) return;
        afterMove.run();
    }

    // s the turret has been stopped (and cond true), 0 if it isn't
    private double stillFor(double t, boolean cond) {
        boolean still = cond && Math.abs(turret.getVelocity()) < STILL_VELOCITY;
        if (!still) stillSince = -1;
        else if (stillSince < 0) stillSince = t;
        return still ? t - stillSince : 0;
    }

    // ---------------------------------------------------------------- feedforward sweep

    private double lowerEnd() {
        return Turret.minTicks + edgeMargin;
    }

    private double upperEnd() {
        return Turret.maxTicks - edgeMargin;
    }

    private int sweepCount() {
        return Math.max(2, sweepSteps);
    }

    private double sweepPower(int runIndex) {
        return sweepStartPower + (sweepEndPower - sweepStartPower) * (runIndex / 2) / (sweepCount() - 1);
    }

    private void startSweep() {
        sweepPoints.clear();
        sweepLog.clear();
        timeConstants.clear();
        timeConstant = Double.NaN;
        fitKS = Double.NaN;
        fitKV = Double.NaN;
        fitError = Double.NaN;
        ranOutOfTravel = false;
        absCheck = Double.NaN;
        sweepRun = 0;
        moveTo(upperEnd(), this::startSweepRun); // old gains, it just needs to get there
    }

    private void startSweepRun() {
        // always head for the far end, so a run that didn't move doesn't leave the next one with no room
        double position = turret.getPosition();
        sweepSign = position - lowerEnd() > upperEnd() - position ? -1 : 1;
        sweepStartPos = position;
        turret.setOpenLoopPower(sweepSign * sweepPower(sweepRun));
        enter(Stage.SWEEP_RUN);
    }

    private void updateSweepRun(double t) {
        double position = turret.getPosition();
        double speed = sweepSign * turret.getVelocity();
        double travel = sweepSign * (position - sweepStartPos);
        samples.add(new double[] {t, speed});

        if (t > 0.4 && travel < -10) {
            fail("turret moved opposite to the power it was given. the motor encoder isn't on the turret motor's port "
                    + "(motorReversed flips both together, so it can't cause this)");
            return;
        }

        double lead = Math.max(0, speed) * brakeLead;
        boolean brake = sweepSign < 0 ? position <= lowerEnd() + lead : position >= upperEnd() - lead;
        boolean stalled = t > STALL_TIME && travel < 5;
        if (!brake && !stalled && t < SWEEP_RUN_TIMEOUT) return;

        turret.stop(); // BRAKE zero power behavior stops it
        double power = sweepPower(sweepRun);
        String dir = sweepSign > 0 ? "left " : "right";

        if (stalled) {
            sweepLog.add(String.format(Locale.US, "%.2f %s -> didn't move (under kS)", power, dir));
        } else {
            double now = mean(t - STEADY_WINDOW, t);
            double before = mean(t - 2 * STEADY_WINDOW, t - STEADY_WINDOW);
            boolean steady = t > 2 * STEADY_WINDOW + 0.1 && Math.abs(now - before) < 0.05 * Math.abs(now) + 10;
            if (steady) {
                sweepPoints.add(new double[] {now, power, sweepSign});
                recordTimeConstant(now);
                sweepLog.add(String.format(Locale.US, "%.2f %s -> %.0f ticks/s (%.0f deg/s)", power, dir, now,
                        Turret.ticksToDegrees(now)));
            } else {
                ranOutOfTravel = true;
                sweepLog.add(String.format(Locale.US, "%.2f %s -> still speeding up at the end, left out", power, dir));
            }
        }
        enter(Stage.SWEEP_STOP);
    }

    private void updateSweepStop(double t) {
        if (stillFor(t, true) < STILL_CONFIRM && t < 2) return;

        sweepRun++;
        // past the power where it couldn't reach a steady speed, bigger ones won't either
        if (sweepRun < 2 * sweepCount() && !(ranOutOfTravel && sweepRun % 2 == 0)) {
            startSweepRun();
            return;
        }
        finishSweep();
    }

    // time from the start of the run to 63% of the speed it settled at
    private void recordTimeConstant(double settled) {
        for (double[] s : samples) {
            if (s[1] >= 0.632 * settled) {
                timeConstants.add(s[0]);
                break;
            }
        }
        if (timeConstants.isEmpty()) return;
        List<Double> sorted = new ArrayList<>(timeConstants);
        Collections.sort(sorted);
        timeConstant = sorted.get(sorted.size() / 2);
    }

    private double mean(double from, double to) {
        double sum = 0;
        int n = 0;
        for (double[] s : samples) {
            if (s[0] <= from || s[0] > to) continue;
            sum += s[1];
            n++;
        }
        return n > 0 ? sum / n : Double.NaN;
    }

    private void finishSweep() {
        if (ranOutOfTravel)
            warnings.add("some sweep powers never reached a steady speed before the end of the swing, lower sweepEndPower "
                    + "if most of them did");

        double[] all = fit(0);
        if (all == null || all[1] <= 0) {
            fail("not enough sweep powers moved the turret at a steady speed to fit kS / kV. raise sweepStartPower / "
                    + "sweepEndPower, or check the turret isn't binding");
            return;
        }
        fitKS = all[0];
        fitKV = all[1];
        fitError = all[2];

        double[] left = fit(1), right = fit(-1);
        if (left != null && right != null) {
            if (Math.abs(left[0] - right[0]) > 0.05 || Math.abs(left[1] - right[1]) > 0.2 * fitKV)
                warnings.add(String.format(Locale.US, "left and right differ (kS %.3f / %.3f, kV %.6f / %.6f), something "
                        + "drags more one way (cable?)", left[0], right[0], left[1], right[1]));
        }
        if (fitError > 0.02)
            warnings.add(String.format(Locale.US, "sweep isn't a clean line (fit error %.3f power), check for binding",
                    fitError));

        // the motor encoder and abs encoder should still agree after all that driving
        turret.readAbsolute();
        absCheck = turret.getAbsoluteCheck();
        if (Math.abs(absCheck) > 3)
            warnings.add(String.format(Locale.US, "abs encoder disagrees with the motor encoder by %.1f deg after the "
                    + "sweep. a belt slipped or gearRatio / encoderGearRatio is wrong, check in TurretTester", absCheck));

        Turret.kS = Math.max(0, fitKS);
        Turret.kV = fitKV;

        if (run == Run.FEEDFORWARD) finish();
        else startPDSearch();
    }

    // least squares |power| = kS + kV * speed. sign 0 = both directions. {kS, kV, rms error} or null
    private double[] fit(double sign) {
        double n = 0, sumV = 0, sumP = 0, sumVV = 0, sumVP = 0;
        for (double[] p : sweepPoints) {
            if (p[0] < MIN_SPEED || (sign != 0 && p[2] != sign)) continue;
            n++;
            sumV += p[0];
            sumP += p[1];
            sumVV += p[0] * p[0];
            sumVP += p[0] * p[1];
        }
        double denominator = n * sumVV - sumV * sumV;
        if (n < 2 || Math.abs(denominator) < 1e-9) return null;

        double kV = (n * sumVP - sumV * sumP) / denominator;
        double kS = (sumP - kV * sumV) / n;
        double squareSum = 0;
        for (double[] p : sweepPoints) {
            if (p[0] < MIN_SPEED || (sign != 0 && p[2] != sign)) continue;
            double r = p[1] - (kS + kV * p[0]);
            squareSum += r * r;
        }
        return new double[] {kS, kV, Math.sqrt(squareSum / n)};
    }

    // ---------------------------------------------------------------- step tests

    private double targetA() {
        return center + stepSize / 2;
    }

    private double targetB() {
        return center - stepSize / 2;
    }

    private void startSteps(Purpose p, double[] targets, double holdTime, double settledBand) {
        purpose = p;
        stepTargets = targets;
        stepIndex = 0;
        hold = holdTime;
        band = settledBand;
        stepResults.clear();
        startStep();
    }

    private void startStep() {
        step = new StepResult();
        step.start = turret.getPosition();
        step.target = stepTargets[stepIndex];
        turret.setTarget(step.target);
        enter(Stage.STEPS);
    }

    private void updateSteps(double t) {
        double error = step.target - turret.getPosition();
        double direction = Math.signum(step.target - step.start);
        step.overshoot = Math.max(step.overshoot, -error * direction);

        if (Math.abs(error) > band) {
            int sign = (int) Math.signum(error);
            if (step.lastSign != 0 && sign != step.lastSign) step.flips++;
            step.lastSign = sign;
            step.insideSince = -1;
        } else if (step.insideSince < 0) {
            step.insideSince = t;
        }

        if (t > hold - FINAL_WINDOW) {
            double v = turret.getVelocity();
            step.errorSum += error;
            step.speedSum += v;
            step.speedSquareSum += v * v;
            step.finalCount++;
        }
        if (t < hold) return;

        if (step.insideSince >= 0 && t - step.insideSince >= SETTLE_CONFIRM) step.settle = step.insideSince;
        if (step.finalCount > 0) {
            step.steadyError = step.errorSum / step.finalCount;
            double meanV = step.speedSum / step.finalCount;
            step.noise = Math.sqrt(Math.max(0, step.speedSquareSum / step.finalCount - meanV * meanV));
        }
        stepResults.add(step);

        stepIndex++;
        if (stepIndex < stepTargets.length) {
            startStep();
            return;
        }
        switch (purpose) {
            case PD: scorePDTest(); break;
            case KS: scoreKSTest(); break;
            default: finishValidation(); break;
        }
    }

    // ---------------------------------------------------------------- PD search

    private void startPDSearch() {
        tau = Double.isNaN(timeConstant) ? defaultTimeConstant : timeConstant;
        if (Double.isNaN(timeConstant))
            warnings.add(String.format(Locale.US, "no sweep this run, using defaultTimeConstant %.2f s for tau", tau));
        pdResults.clear();
        pickedPD = null;
        refining = false;
        // wn = 1 / tau: kP is gentle and kD comes out ~kV at zeta 1
        startPDTest(1 / tau, zeta);
    }

    private void startPDTest(double wn, double z) {
        currentPD = new PDTest();
        currentPD.wn = wn;
        currentPD.zeta = z;
        currentPD.kP = wn * wn * tau * Turret.kV;
        currentPD.kD = Math.max(minKD, (2 * z * wn * tau - 1) * Turret.kV);
        Turret.kP = currentPD.kP;
        Turret.kD = currentPD.kD;
        // get to A with these gains first, then score A -> B -> A
        moveTo(targetA(), () -> startSteps(Purpose.PD, new double[] {targetB(), targetA()}, stepHold, settleBand));
    }

    private void scorePDTest() {
        PDTest r = currentPD;
        boolean settled = true;
        double settle = 0;
        for (StepResult s : stepResults) {
            r.overshoot = Math.max(r.overshoot, s.overshoot);
            r.flips = Math.max(r.flips, s.flips);
            r.noise = Math.max(r.noise, s.noise);
            if (Math.abs(s.steadyError) > Math.abs(r.steadyError)) r.steadyError = s.steadyError;
            if (Double.isNaN(s.settle)) settled = false;
            else settle = Math.max(settle, s.settle);
        }
        if (settled) r.settle = settle;

        boolean oscillating = r.flips >= 2;
        boolean overshoots = r.overshoot > overshootLimit;
        boolean buzzing = r.noise > buzzVelocity;
        r.ok = !oscillating && !overshoots && !buzzing;
        r.verdict = oscillating ? "OSCILLATING" : overshoots ? "OVERSHOOTS" : buzzing ? "BUZZING"
                : settled ? "ok" : "slow / stops short";
        pdResults.add(r);

        if (refining) {
            nextRefine();
            return;
        }
        int grown = 0;
        for (PDTest p : pdResults) if (p.zeta == zeta) grown++;
        if (r.ok && grown < pdTests) startPDTest(r.wn * wnRatio, zeta);
        else pickWn();
    }

    private void pickWn() {
        double fastest = Double.POSITIVE_INFINITY;
        for (PDTest r : pdResults) if (r.ok && !Double.isNaN(r.settle)) fastest = Math.min(fastest, r.settle);

        pickedPD = null;
        if (fastest < Double.POSITIVE_INFINITY) {
            for (PDTest r : pdResults) {
                if (r.ok && !Double.isNaN(r.settle) && r.settle <= fastest * (1 + settleMargin) + 0.02) {
                    pickedPD = r;
                    break;
                }
            }
        } else {
            for (PDTest r : pdResults) if (r.ok) pickedPD = r; // none settled, take the stiffest stable one
            if (pickedPD != null)
                warnings.add("no PD test settled inside settleBand, friction is probably holding it short. "
                        + "the kS check should fix that, if not raise settleBand or stepHold");
        }

        if (pickedPD == null) {
            fail(String.format(Locale.US, "even the gentlest gains were %s. likely gear slop / backlash, try a bigger "
                    + "zeta or minKD", pdResults.get(0).verdict));
            return;
        }
        PDTest last = pdResults.get(pdResults.size() - 1);
        if (last.ok && pdResults.size() >= pdTests)
            warnings.add("still stable at the last wn tested, raise pdTests to try stiffer gains");

        // a little less and more damping at the picked stiffness, slop sometimes wants more than the model says
        refining = true;
        refineZetas.clear();
        refineZetas.add(zeta * 0.75);
        refineZetas.add(zeta * 1.4);
        nextRefine();
    }

    private void nextRefine() {
        if (!refineZetas.isEmpty()) {
            startPDTest(pickedPD.wn, refineZetas.remove(0));
            return;
        }
        PDTest best = pickedPD;
        for (PDTest r : pdResults) {
            if (r.wn != pickedPD.wn || r == pickedPD || !r.ok || Double.isNaN(r.settle)) continue;
            if (Double.isNaN(best.settle) || r.settle < best.settle - 0.02) best = r;
        }
        pickedPD = best;
        Turret.kP = best.kP;
        Turret.kD = best.kD;
        startKSCheck();
    }

    // ---------------------------------------------------------------- kS check

    private void startKSCheck() {
        kSResults.clear();
        kSLow = 0;
        kSHigh = Double.POSITIVE_INFINITY;
        startKSTest(Turret.kS);
    }

    private void startKSTest(double kS) {
        Turret.kS = kS;
        double up = center + smallStep;
        moveTo(center, () -> startSteps(Purpose.KS, new double[] {up, center, up, center}, smallStepHold,
                Turret.tolerance));
    }

    private void scoreKSTest() {
        KSTest r = new KSTest();
        r.kS = Turret.kS;
        r.steps = stepResults.size();
        for (StepResult s : stepResults) {
            if (Math.abs(s.steadyError) < Turret.tolerance) r.reached++;
            if (s.flips >= 2 || s.noise > buzzVelocity) r.hunting = true;
        }
        kSResults.add(r);

        boolean good = r.reached == r.steps && !r.hunting;
        if (good) {
            startValidation();
            return;
        }
        if (r.hunting) kSHigh = Math.min(kSHigh, r.kS);
        else kSLow = Math.max(kSLow, r.kS);

        if (kSResults.size() >= kSTries) {
            // best one that didn't hunt, most steps reached
            KSTest best = null;
            for (KSTest k : kSResults)
                if (!k.hunting && (best == null || k.reached > best.reached)) best = k;
            Turret.kS = best != null ? best.kS : kSLow;
            warnings.add("kS check never got every small step inside tolerance without hunting. the gear slop may be "
                    + "bigger than Turret.tolerance, try a bigger tolerance");
            startValidation();
            return;
        }

        double next = kSHigh == Double.POSITIVE_INFINITY
                ? Math.max(kSLow * 1.25, kSLow + 0.01)
                : (kSLow + kSHigh) / 2;
        startKSTest(next);
    }

    // ---------------------------------------------------------------- validation

    private void startValidation() {
        double kSBefore = Double.isNaN(fitKS) ? savedKS : Math.max(0, fitKS);
        if (kSBefore > 0 && Math.abs(Turret.kS - kSBefore) > 0.25 * kSBefore)
            warnings.add("kS moved a lot in the kS check, run dpad right once more so the PD search uses it");
        moveTo(targetA(), () -> startSteps(Purpose.VALIDATE, new double[] {targetB(), targetA(),
                center + bigStep / 2, center - bigStep / 2}, stepHold, Turret.tolerance));
    }

    private void finishValidation() {
        validation.clear();
        validation.addAll(stepResults);
        StepResult big = validation.get(validation.size() - 1);
        if (big.overshoot > 2 * overshootLimit)
            warnings.add(String.format(Locale.US, "the %.0f tick step overshot %.0f ticks, lower Turret.wrapPower",
                    bigStep, big.overshoot));
        finish();
    }

    // ---------------------------------------------------------------- telemetry

    private void addTelemetry() {
        // numbers first so they graph
        joinedTelemetry.addData("Target", turret.isOpenLoop() ? turret.getPosition() : turret.getTarget());
        joinedTelemetry.addData("Position", turret.getPosition());
        joinedTelemetry.addData("Error", turret.isOpenLoop() ? 0 : turret.getError());
        joinedTelemetry.addData("Power", turret.getPower());
        joinedTelemetry.addData("Velocity", turret.getVelocity());
        joinedTelemetry.addData("Angle (deg)", turret.getAngle());
        joinedTelemetry.addLine();

        joinedTelemetry.addData("Stage", describeStage());
        if (!status.isEmpty()) joinedTelemetry.addLine(status);
        for (String warning : warnings) joinedTelemetry.addLine("! " + warning);
        joinedTelemetry.addLine();

        if (finished) {
            joinedTelemetry.addLine("=== copy into Turret.java ===");
            joinedTelemetry.addLine(String.format(Locale.US, "public static double kP = %.5f;", Turret.kP));
            joinedTelemetry.addLine(String.format(Locale.US, "public static double kD = %.6f;", Turret.kD));
            joinedTelemetry.addLine(String.format(Locale.US, "public static double kS = %.4f;", Turret.kS));
            joinedTelemetry.addLine(String.format(Locale.US, "public static double kV = %.6f;", Turret.kV));
            joinedTelemetry.addLine("(already live in Turret, they reset when the opmode restarts)");
        } else {
            joinedTelemetry.addData("Gains", String.format(Locale.US, "kP %.5f  kD %.6f  kS %.4f  kV %.6f",
                    Turret.kP, Turret.kD, Turret.kS, Turret.kV));
        }
        joinedTelemetry.addLine();

        if (!validation.isEmpty()) {
            joinedTelemetry.addLine("--- validation: step | settle | overshoot | steady error (ticks) ---");
            for (StepResult s : validation) {
                joinedTelemetry.addLine(String.format(Locale.US, "%.0f -> %.0f | %s | %.1f | %.1f",
                        s.start, s.target, Double.isNaN(s.settle) ? "never" : String.format(Locale.US, "%.2f s", s.settle),
                        s.overshoot, s.steadyError));
            }
            joinedTelemetry.addLine("goal: settle < 0.3 s on the 91 tick steps, overshoot < ~4, steady error < tolerance");
            joinedTelemetry.addLine();
        }

        if (!kSResults.isEmpty()) {
            joinedTelemetry.addLine("--- kS check: kS | small steps reached | hunting ---");
            for (KSTest k : kSResults)
                joinedTelemetry.addLine(String.format(Locale.US, "%.4f | %d / %d | %s", k.kS, k.reached, k.steps,
                        k.hunting ? "HUNTING" : "no"));
            joinedTelemetry.addLine();
        }

        if (!pdResults.isEmpty()) {
            joinedTelemetry.addLine(String.format(Locale.US, "--- PD tests (tau %.3f s): wn | zeta | kP | kD | settle | "
                    + "overshoot | noise ---", tau));
            for (PDTest r : pdResults) {
                joinedTelemetry.addLine(String.format(Locale.US, "%.1f | %.2f | %.5f | %.6f | %s | %.1f | %.0f  %s%s",
                        r.wn, r.zeta, r.kP, r.kD,
                        Double.isNaN(r.settle) ? "-" : String.format(Locale.US, "%.2f s", r.settle),
                        r.overshoot, r.noise, r.verdict, r == pickedPD ? "  <- PICKED" : ""));
            }
            joinedTelemetry.addLine();
        }

        if (!sweepLog.isEmpty()) {
            joinedTelemetry.addLine("--- feedforward sweep: power -> steady speed ---");
            for (String line : sweepLog) joinedTelemetry.addLine(line);
            if (!Double.isNaN(fitKV))
                joinedTelemetry.addLine(String.format(Locale.US, "kS %.4f  kV %.6f  fit error %.3f power  "
                        + "(top speed ~%.0f ticks/s)", fitKS, fitKV, fitError, (1 - fitKS) / fitKV));
            if (!Double.isNaN(timeConstant))
                joinedTelemetry.addLine(String.format(Locale.US, "time constant %.3f s", timeConstant));
            if (!Double.isNaN(absCheck))
                joinedTelemetry.addLine(String.format(Locale.US, "abs encoder check %.2f deg (should be ~0)", absCheck));
            joinedTelemetry.addLine();
        }

        joinedTelemetry.addLine("a = auto tune  dpad up = kS/kV only  dpad right = PD + kS only  x = hold  b = stop");
        joinedTelemetry.update();
    }

    private String describeStage() {
        switch (stage) {
            case MOVE: return String.format(Locale.US, "moving to %.0f", turret.getTarget());
            case SWEEP_RUN: return String.format(Locale.US, "feedforward sweep: power %.2f %s (%d / %d)",
                    sweepPower(sweepRun), sweepSign > 0 ? "left" : "right", sweepRun + 1, 2 * sweepCount());
            case SWEEP_STOP: return "feedforward sweep: stopping";
            case STEPS:
                String what = purpose == Purpose.PD
                        ? String.format(Locale.US, "PD test %d (wn %.1f zeta %.2f)", pdResults.size() + 1,
                                currentPD.wn, currentPD.zeta)
                        : purpose == Purpose.KS ? String.format(Locale.US, "kS check %d (kS %.4f)",
                                kSResults.size() + 1, Turret.kS)
                        : "validation";
                return String.format(Locale.US, "%s: step %d / %d", what, stepIndex + 1, stepTargets.length);
            case HOLD: return String.format(Locale.US, "holding %.0f (x or b to stop)", center);
            default: return "idle, press a to auto tune";
        }
    }

    private double batteryVoltage() {
        double lowest = Double.NaN;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0 && (Double.isNaN(lowest) || voltage < lowest)) lowest = voltage;
        }
        return lowest;
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
