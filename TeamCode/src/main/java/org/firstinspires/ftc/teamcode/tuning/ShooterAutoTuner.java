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

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;



// automatically tunes the flywheel velocity controller in Shooter (kS, kV, kP)
// put the robot on a charged battery, keep balls out of the shooter, press a and leave it alone for about a minute.
// when it's done it prints the three lines to paste into Shooter.java (they're already applied live until restart)

// a = full auto tune   dpad up = feedforward only (kS, kV)   dpad right = kP only (keeps the current kS, kV)
// x = hold testVelocity with the current gains   b = stop / abort (abort puts the old gains back)
// graph Target, Velocity and Power in panels / dashboard to watch it work

// what it does:
// 1. feedforward sweep: runs fixed powers with no PID, waits for the flywheel to settle at each one and fits a line
//    power = kS + kV * velocity through them. also measures how slow the flywheel is (time constant)
// 2. kP search: holds testVelocity with bigger and bigger kP, starting at kV. for each one it cuts the power to fake
//    a shot (velocity drops kickDrop), then times how long it takes to get back inside Shooter.tolerance.
//    it stops as soon as one oscillates, overshoots or chatters the power, then picks the smallest kP that recovers
//    about as fast as the fastest one. more kP past that is just twitchier, it can't recover any faster once the
//    motor is already at full power
// 3. holds testVelocity with the new gains. feed a few real balls through and check DIP / RECOVERY
// there's no kI or kD: with a fitted kV the feedforward already lands on the target, kI would only wind up during
// spin up, and kD on a noisy velocity reading just adds chatter. kS + kV + kP is all the flywheel needs
@Configurable
@Config
@TeleOp
public class ShooterAutoTuner extends OpMode {
    public static double testVelocity = 1300; // ticks/s the kP search and hold run at, use a typical shot speed

    // feedforward sweep
    public static double sweepStartPower = 0.2;
    public static double sweepEndPower = 0.9;
    public static int sweepSteps = 8;

    // kP search
    public static int kPTests = 12;               // most kP values it tries, ~3 s each
    public static double kPRatio = 1.6;           // each kP is this many times the last
    public static double kickDrop = 150;          // ticks/s the fake shot pulls the flywheel down
    public static double measureTime = 1.5;       // seconds held at testVelocity to measure noise + steady error
    public static double oscillationFactor = 2.5; // velocity noise this many times the quietest kP = oscillating
    public static double overshootLimit = 0.5;    // fraction of Shooter.tolerance it may overshoot after a shot
    public static double recoveryMargin = 0.15;   // picks the smallest kP within this fraction of the fastest recovery

    private static final double STEADY_WINDOW = 0.4;  // s, two windows in a row averaging the same speed = steady
    private static final double SETTLE_CONFIRM = 0.3; // s it has to stay inside tolerance to count as settled
    private static final double KICK_MAX_TIME = 0.6;  // s, longest the power is cut for a fake shot
    private static final double COAST_VELOCITY = 30;  // ticks/s, slower than this counts as stopped
    private static final double COAST_TIMEOUT = 15;
    private static final double LOW_BATTERY = 12.5;

    private enum Stage { IDLE, COAST, SWEEP, KP_SETTLE, KP_MEASURE, KP_KICK, KP_RECOVER, HOLD }
    private enum Run { FULL, FEEDFORWARD, KP }

    private static class KPTest {
        double kP;
        double steadyError, noise; // mean and std dev of the error while holding, ticks/s
        double clipped;            // fraction of the hold with power stuck at 0 or 1
        double dip, overshoot;     // ticks/s, from the fake shot
        double recovery = Double.NaN; // s from releasing the fake shot to back inside tolerance, NaN = never
        boolean ok;
        String verdict;
    }

    private Shooter shooter;
    private JoinedTelemetry joinedTelemetry;

    private Stage stage = Stage.IDLE;
    private Run run = Run.FULL;
    private final ElapsedTime stageTimer = new ElapsedTime();
    private final List<double[]> samples = new ArrayList<>(); // {t, velocity} since the sweep step started

    private double savedKS, savedKV, savedKP; // put back on abort / fail
    private String status = "";
    private final List<String> warnings = new ArrayList<>();
    private boolean finished = false;

    // feedforward sweep
    private int sweepIndex = 0;
    private final List<double[]> sweepPoints = new ArrayList<>(); // {velocity, power}
    private final List<Double> timeConstants = new ArrayList<>();
    private double timeConstant = Double.NaN;
    private double voltageSum = 0;
    private int voltageCount = 0;
    private double fitKS = Double.NaN, fitKV = Double.NaN, fitError = Double.NaN;

    // kP search
    private final List<KPTest> kPTestResults = new ArrayList<>();
    private KPTest current;
    private KPTest picked;
    private double quietestNoise;
    private double insideSince = -1;
    private double errorSum, errorSquareSum;
    private int measureCount, clipCount;
    private double kickStart;

    // real shots while holding
    private boolean holdSettled = false, inDip = false;
    private double dipStart = 0, dipMin = 0;
    private double lastDip = Double.NaN, lastRecovery = Double.NaN;

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
        joinedTelemetry.addLine("keep balls out of the shooter, press start then a for the full auto tune (~1 min)");
        double voltage = batteryVoltage();
        joinedTelemetry.addData("Battery", String.format(Locale.US, "%.1f V%s", voltage,
                voltage < LOW_BATTERY ? "  (low, charge it first, kV depends on it)" : ""));
        joinedTelemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) startRun(Run.FULL);
        if (gamepad1.dpadUpWasPressed()) startRun(Run.FEEDFORWARD);
        if (gamepad1.dpadRightWasPressed()) startRun(Run.KP);
        if (gamepad1.xWasPressed()) toggleHold();
        if (gamepad1.bWasPressed()) abort();

        double t = stageTimer.seconds();
        double velocity = shooter.getVelocity();

        switch (stage) {
            case COAST: updateCoast(t, velocity); break;
            case SWEEP: updateSweep(t, velocity); break;
            case KP_SETTLE: updateKPSettle(t, velocity); break;
            case KP_MEASURE: updateKPMeasure(t, velocity); break;
            case KP_KICK: updateKPKick(t, velocity); break;
            case KP_RECOVER: updateKPRecover(t, velocity); break;
            case HOLD: updateHold(t, velocity); break;
            default: break;
        }

        shooter.periodic();
        addTelemetry(velocity);
    }

    // ---------------------------------------------------------------- runs

    private void startRun(Run r) {
        if (running()) return; // b first to abort

        run = r;
        savedKS = Shooter.kS;
        savedKV = Shooter.kV;
        savedKP = Shooter.kP;
        status = "";
        warnings.clear();
        finished = false;
        kPTestResults.clear();
        picked = null;
        lastDip = Double.NaN;
        lastRecovery = Double.NaN;

        double voltage = batteryVoltage();
        if (voltage < LOW_BATTERY)
            warnings.add(String.format(Locale.US, "battery is %.1f V, kS / kV come out different on a charged one", voltage));

        if (r == Run.KP) {
            startKPSearch();
            return;
        }

        sweepPoints.clear();
        timeConstants.clear();
        timeConstant = Double.NaN;
        voltageSum = 0;
        voltageCount = 0;
        fitKS = Double.NaN;
        fitKV = Double.NaN;
        fitError = Double.NaN;
        shooter.stop();
        enter(Stage.COAST);
    }

    private boolean running() {
        return stage != Stage.IDLE && stage != Stage.HOLD;
    }

    private void enter(Stage next) {
        stage = next;
        stageTimer.reset();
        samples.clear();
        insideSince = -1;
    }

    private void abort() {
        if (running()) {
            restoreGains();
            status = "aborted, old gains put back";
        }
        shooter.stop();
        enter(Stage.IDLE);
    }

    private void fail(String message) {
        restoreGains();
        shooter.stop();
        status = "FAILED: " + message;
        enter(Stage.IDLE);
    }

    private void restoreGains() {
        Shooter.kS = savedKS;
        Shooter.kV = savedKV;
        Shooter.kP = savedKP;
    }

    private void finish() {
        finished = true;
        status = "done! hold is running, feed a few balls to check it (b to stop)";
        startHold();
    }

    // ---------------------------------------------------------------- feedforward sweep

    private void updateCoast(double t, double velocity) {
        if (Math.abs(velocity) > COAST_VELOCITY && t < COAST_TIMEOUT) return;
        sweepIndex = 0;
        startSweepStep();
    }

    private void startSweepStep() {
        shooter.setOpenLoopPower(sweepPower(sweepIndex));
        enter(Stage.SWEEP);
    }

    private void updateSweep(double t, double velocity) {
        samples.add(new double[] {t, velocity});

        // waits longer once it knows how slow the flywheel is
        double minTime = Double.isNaN(timeConstant) ? 1 : 2 * timeConstant;
        double maxTime = Double.isNaN(timeConstant) ? 10 : 6 * timeConstant + 2;
        if (!steady(t, minTime, maxTime)) return;

        double settledVelocity = mean(t - STEADY_WINDOW, t);
        if (settledVelocity < -100) {
            fail("velocity reads negative at positive power, the encoder in sTop's port is counting backwards "
                    + "(is it plugged into the right motor?)");
            return;
        }
        sweepPoints.add(new double[] {settledVelocity, sweepPower(sweepIndex)});
        recordTimeConstant(settledVelocity);
        voltageSum += batteryVoltage();
        voltageCount++;

        sweepIndex++;
        if (sweepIndex < sweepCount()) {
            startSweepStep();
            return;
        }

        shooter.stop();
        double fastest = 0;
        for (double[] p : sweepPoints) fastest = Math.max(fastest, p[0]);
        if (fastest < 100) {
            fail(String.format(Locale.US, "no speed at %.2f power. check the flywheel actually spins and the sTop "
                    + "encoder cable is plugged in", sweepEndPower));
            return;
        }

        fitFeedforward();
        if (Double.isNaN(fitKV) || fitKV <= 0) {
            fail("not enough sweep powers moved the flywheel to fit kS / kV, raise sweepStartPower and sweepEndPower");
            return;
        }
        if (fitError > 0.02)
            warnings.add(String.format(Locale.US, "sweep isn't a clean line (fit error %.3f power), check for rubbing "
                    + "or a sagging battery", fitError));

        Shooter.kS = fitKS;
        Shooter.kV = fitKV;

        if (run == Run.FEEDFORWARD) finish();
        else startKPSearch();
    }

    // time for this step to get 63% of the way from where it started to where it settled
    private void recordTimeConstant(double settledVelocity) {
        double start = samples.get(0)[1];
        double change = settledVelocity - start;
        if (Math.abs(change) < 30) return;

        double threshold = start + 0.632 * change;
        for (double[] s : samples) {
            if ((s[1] - threshold) * Math.signum(change) >= 0) {
                timeConstants.add(s[0]);
                break;
            }
        }
        if (timeConstants.isEmpty()) return;

        List<Double> sorted = new ArrayList<>(timeConstants);
        Collections.sort(sorted);
        timeConstant = sorted.get(sorted.size() / 2);
    }

    // true once the last two windows average the same speed (or it's waited maxTime)
    private boolean steady(double t, double minTime, double maxTime) {
        if (t < Math.max(minTime, 2 * STEADY_WINDOW)) return false;
        if (t > maxTime) return true;
        double now = mean(t - STEADY_WINDOW, t);
        double before = mean(t - 2 * STEADY_WINDOW, t - STEADY_WINDOW);
        return Math.abs(now - before) < 0.005 * Math.abs(now) + 3;
    }

    private double mean(double from, double to) {
        double sum = 0;
        int n = 0;
        for (double[] s : samples) {
            if (s[0] < from || s[0] > to) continue;
            sum += s[1];
            n++;
        }
        return n > 0 ? sum / n : Double.NaN;
    }

    private int sweepCount() {
        return Math.max(2, sweepSteps);
    }

    private double sweepPower(int i) {
        return sweepStartPower + (sweepEndPower - sweepStartPower) * i / (sweepCount() - 1);
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

        double squareSum = 0;
        for (double[] p : sweepPoints) {
            if (p[0] < 50) continue;
            double residual = p[1] - (fitKS + fitKV * p[0]);
            squareSum += residual * residual;
        }
        fitError = Math.sqrt(squareSum / n);
    }

    // ---------------------------------------------------------------- kP search

    private void startKPSearch() {
        if (Shooter.kV <= 0) {
            fail("kV is 0, run the feedforward sweep first (dpad up or a)");
            return;
        }
        double fullPowerVelocity = (1 - Shooter.kS) / Shooter.kV;
        if (testVelocity > 0.85 * fullPowerVelocity)
            warnings.add(String.format(Locale.US, "testVelocity %.0f is close to full power (~%.0f), the motor has no "
                    + "headroom to recover. lower testVelocity", testVelocity, fullPowerVelocity));

        kPTestResults.clear();
        picked = null;
        quietestNoise = Double.POSITIVE_INFINITY;
        startKPTest(Shooter.kV); // kP = kV doubles how fast it closes the gap vs feedforward alone, a gentle start
    }

    private void startKPTest(double kP) {
        current = new KPTest();
        current.kP = kP;
        Shooter.kP = kP;
        shooter.setTargetVelocity(testVelocity);
        enter(Stage.KP_SETTLE);
    }

    // timeouts scale with how slow the flywheel is, 1 s if the sweep didn't run
    private double timeout() {
        return 2 + 3 * (Double.isNaN(timeConstant) ? 1 : timeConstant);
    }

    private boolean updateInside(double t, double velocity) {
        boolean inside = Math.abs(testVelocity - velocity) < Shooter.tolerance;
        if (!inside) insideSince = -1;
        else if (insideSince < 0) insideSince = t;
        return inside && t - insideSince > SETTLE_CONFIRM;
    }

    private void updateKPSettle(double t, double velocity) {
        // measures even if it never settles, an oscillating kP still needs to be scored as one
        if (!updateInside(t, velocity) && t < timeout()) return;
        errorSum = 0;
        errorSquareSum = 0;
        measureCount = 0;
        clipCount = 0;
        enter(Stage.KP_MEASURE);
    }

    private void updateKPMeasure(double t, double velocity) {
        double error = testVelocity - velocity;
        errorSum += error;
        errorSquareSum += error * error;
        measureCount++;
        if (shooter.getPower() <= 0 || shooter.getPower() >= 1) clipCount++;
        if (t < measureTime) return;

        current.steadyError = errorSum / measureCount;
        current.noise = Math.sqrt(Math.max(0, errorSquareSum / measureCount - current.steadyError * current.steadyError));
        current.clipped = (double) clipCount / measureCount;

        // fake shot: cut the power and let it coast down
        kickStart = velocity;
        shooter.setOpenLoopPower(0);
        enter(Stage.KP_KICK);
    }

    private void updateKPKick(double t, double velocity) {
        if (velocity > kickStart - kickDrop && t < KICK_MAX_TIME) return;
        shooter.setTargetVelocity(testVelocity);
        current.dip = testVelocity - velocity;
        current.overshoot = 0;
        enter(Stage.KP_RECOVER);
    }

    private void updateKPRecover(double t, double velocity) {
        current.dip = Math.max(current.dip, testVelocity - velocity);
        current.overshoot = Math.max(current.overshoot, velocity - testVelocity);
        boolean recovered = updateInside(t, velocity);
        if (!recovered && t < timeout()) return;

        if (recovered) current.recovery = insideSince;
        scoreKPTest();
    }

    private void scoreKPTest() {
        double tolerance = Shooter.tolerance;
        boolean oscillating = current.noise > oscillationFactor * quietestNoise && current.noise > 0.3 * tolerance;
        boolean overshoots = current.overshoot > overshootLimit * tolerance;
        boolean chatters = current.clipped > 0.3;

        current.ok = !oscillating && !overshoots && !chatters;
        current.verdict = oscillating ? "OSCILLATING"
                : overshoots ? "OVERSHOOTS"
                : chatters ? "POWER CHATTERS"
                : Double.isNaN(current.recovery) ? "slow" : "ok";
        kPTestResults.add(current);
        if (current.ok) quietestNoise = Math.min(quietestNoise, current.noise);

        if (current.ok && kPTestResults.size() < kPTests) startKPTest(current.kP * kPRatio);
        else pickKP();
    }

    private void pickKP() {
        double fastest = Double.POSITIVE_INFINITY;
        for (KPTest r : kPTestResults)
            if (r.ok && !Double.isNaN(r.recovery)) fastest = Math.min(fastest, r.recovery);

        picked = null;
        if (fastest < Double.POSITIVE_INFINITY) {
            // smallest kP that's about as quick as the quickest
            for (KPTest r : kPTestResults) {
                if (r.ok && !Double.isNaN(r.recovery) && r.recovery <= fastest * (1 + recoveryMargin) + 0.02) {
                    picked = r;
                    break;
                }
            }
        } else {
            // nothing got back in time, take the biggest one that was still stable
            for (KPTest r : kPTestResults) if (r.ok) picked = r;
            if (picked != null)
                warnings.add("no kP recovered from the fake shot in time, try a lower kickDrop or more kPTests");
        }

        if (picked == null) {
            double fallback = kPTestResults.get(0).kP / kPRatio;
            Shooter.kP = fallback;
            warnings.add(String.format(Locale.US, "even the smallest kP was %s, using %.6f. the velocity reading may be "
                    + "noisy, check for a wobbly flywheel or loose encoder cable", kPTestResults.get(0).verdict, fallback));
        } else {
            Shooter.kP = picked.kP;
            if (Math.abs(picked.steadyError) > 0.5 * Shooter.tolerance)
                warnings.add(String.format(Locale.US, "steady error is %.0f ticks/s, kV may be off. rerun the full tune "
                        + "(a) on a charged battery", picked.steadyError));
            if (picked == kPTestResults.get(kPTestResults.size() - 1) && kPTestResults.size() >= kPTests)
                warnings.add("still improving at the last kP tested, raise kPTests to try bigger ones");
        }
        finish();
    }

    // ---------------------------------------------------------------- hold + real shots

    private void toggleHold() {
        if (running()) return;
        if (stage == Stage.HOLD) {
            shooter.stop();
            enter(Stage.IDLE);
        } else {
            startHold();
        }
    }

    private void startHold() {
        shooter.setTargetVelocity(testVelocity);
        holdSettled = false;
        inDip = false;
        enter(Stage.HOLD);
    }

    private void updateHold(double t, double velocity) {
        if (!holdSettled) {
            holdSettled = updateInside(t, velocity);
            return;
        }

        // settled: anything that drops it below the band is a shot going through
        boolean inside = Math.abs(testVelocity - velocity) < Shooter.tolerance;
        if (!inDip && velocity < testVelocity - Shooter.tolerance) {
            inDip = true;
            dipStart = t;
            dipMin = velocity;
        }
        if (inDip) {
            dipMin = Math.min(dipMin, velocity);
            if (inside) {
                inDip = false;
                lastDip = testVelocity - dipMin;
                lastRecovery = t - dipStart;
            }
        }
    }

    // ---------------------------------------------------------------- telemetry

    private void addTelemetry(double velocity) {
        // numbers first so they graph
        joinedTelemetry.addData("Target", shooter.getTargetVelocity());
        joinedTelemetry.addData("Velocity", velocity);
        joinedTelemetry.addData("Error", shooter.getTargetVelocity() - velocity);
        joinedTelemetry.addData("Power", shooter.getPower());
        joinedTelemetry.addLine();

        joinedTelemetry.addData("Stage", describeStage());
        if (!status.isEmpty()) joinedTelemetry.addLine(status);
        for (String warning : warnings) joinedTelemetry.addLine("! " + warning);
        joinedTelemetry.addLine();

        if (finished) {
            joinedTelemetry.addLine("=== copy into Shooter.java ===");
            joinedTelemetry.addLine(String.format(Locale.US, "public static double kS = %.4f;", Shooter.kS));
            joinedTelemetry.addLine(String.format(Locale.US, "public static double kV = %.7f;", Shooter.kV));
            joinedTelemetry.addLine(String.format(Locale.US, "public static double kP = %.6f;", Shooter.kP));
            joinedTelemetry.addLine("(already live in Shooter, they reset when the opmode restarts)");
            joinedTelemetry.addLine();
        } else {
            joinedTelemetry.addData("Gains", String.format(Locale.US, "kS %.4f  kV %.7f  kP %.6f  tolerance %.0f",
                    Shooter.kS, Shooter.kV, Shooter.kP, Shooter.tolerance));
            joinedTelemetry.addLine();
        }

        if (stage == Stage.HOLD) {
            joinedTelemetry.addLine("--- last real shot ---");
            joinedTelemetry.addData("DIP", Double.isNaN(lastDip) ? "-" : String.format(Locale.US, "%.0f ticks/s", lastDip));
            joinedTelemetry.addData("RECOVERY", Double.isNaN(lastRecovery) ? "-"
                    : String.format(Locale.US, "%.2f s  (good is under ~0.3 s)", lastRecovery));
            joinedTelemetry.addLine();
        }

        if (!kPTestResults.isEmpty()) {
            joinedTelemetry.addLine("--- kP tests: kP | recovery | overshoot | noise | steady error ---");
            for (KPTest r : kPTestResults) {
                joinedTelemetry.addLine(String.format(Locale.US, "%.6f | %s | %.0f | %.0f | %.0f  %s%s",
                        r.kP, Double.isNaN(r.recovery) ? "-" : String.format(Locale.US, "%.2f s", r.recovery),
                        r.overshoot, r.noise, r.steadyError, r.verdict, r == picked ? "  <- PICKED" : ""));
            }
            joinedTelemetry.addLine();
        }

        if (!sweepPoints.isEmpty()) {
            joinedTelemetry.addLine("--- feedforward sweep: power -> ticks/s ---");
            for (double[] p : sweepPoints)
                joinedTelemetry.addLine(String.format(Locale.US, "%.2f -> %.0f", p[1], p[0]));
            if (!Double.isNaN(fitKV))
                joinedTelemetry.addLine(String.format(Locale.US, "kS %.4f  kV %.7f  fit error %.3f power",
                        fitKS, fitKV, fitError));
            if (!Double.isNaN(timeConstant))
                joinedTelemetry.addLine(String.format(Locale.US, "time constant %.2f s  battery %.1f V",
                        timeConstant, voltageCount > 0 ? voltageSum / voltageCount : Double.NaN));
            joinedTelemetry.addLine();
        }

        joinedTelemetry.addLine("a = auto tune  dpad up = kS/kV only  dpad right = kP only  x = hold  b = stop");
        joinedTelemetry.update();
    }

    private String describeStage() {
        int test = kPTestResults.size() + 1;
        String kPTest = current == null ? "" : String.format(Locale.US, "kP test %d / %d (kP %.6f): ",
                test, kPTests, current.kP);
        switch (stage) {
            case COAST: return "waiting for the flywheel to stop";
            case SWEEP: return String.format(Locale.US, "feedforward sweep: power %.2f (%d / %d)",
                    sweepPower(sweepIndex), sweepIndex + 1, sweepCount());
            case KP_SETTLE: return kPTest + "settling";
            case KP_MEASURE: return kPTest + "measuring";
            case KP_KICK: return kPTest + "fake shot";
            case KP_RECOVER: return kPTest + "recovering";
            case HOLD: return String.format(Locale.US, "holding %.0f ticks/s (x or b to stop)", testVelocity);
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
        shooter.stop();
    }
}
