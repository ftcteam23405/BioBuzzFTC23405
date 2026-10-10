package org.firstinspires.ftc.teamcode.commandbase.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Reads the output of the "POLLEN DETECTOR v3" Limelight Python SnapScript.
 *
 * llpython layout (32 doubles):
 *   [0]          = number of balls reported (max 6)
 *   [1 + 5*i ..] = tx_deg, ty_deg, radius_px, class (0 = POLLEN, 1 = OTHER), score
 * Pollen always come first in the list.
 */
public class PollenDetector {

    public static class Ball {
        public final double tx, ty, radiusPx, score;
        public final boolean isPollen;

        Ball(double tx, double ty, double radiusPx, double cls, double score) {
            this.tx = tx;
            this.ty = ty;
            this.radiusPx = radiusPx;
            this.isPollen = cls < 0.5;
            this.score = score;
        }
    }

    private static final int FIELDS = 5;
    private static final int MAX_BALLS = 6;
    private static final long MAX_STALENESS_MS = 100;   // ignore results older than this
    private static final int SMOOTH_FRAMES = 5;         // median filter over this many frames

    private final Limelight3A limelight;
    private final int[] countHistory = new int[SMOOTH_FRAMES];
    private int histIdx = 0;

    private List<Ball> balls = new ArrayList<>();
    private boolean fresh = false;

    public PollenDetector(HardwareMap hw, String name) {
        limelight = hw.get(Limelight3A.class, name);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    /** Call once per loop. Returns the balls seen in the latest frame. */
    public List<Ball> update() {
        LLResult r = limelight.getLatestResult();
        fresh = false;
        List<Ball> out = new ArrayList<>();

        if (r != null && r.getStaleness() < MAX_STALENESS_MS) {
            double[] py = r.getPythonOutput();
            if (py != null && py.length >= 1) {
                fresh = true;
                int n = (int) Math.round(py[0]);
                n = Math.max(0, Math.min(n, MAX_BALLS));
                for (int i = 0; i < n; i++) {
                    int b = 1 + FIELDS * i;
                    if (b + FIELDS > py.length) break;
                    out.add(new Ball(py[b], py[b + 1], py[b + 2], py[b + 3], py[b + 4]));
                }
            }
        }

        balls = out;
        countHistory[histIdx] = rawPollenCount();
        histIdx = (histIdx + 1) % SMOOTH_FRAMES;
        return balls;
    }

    /** Pollen seen in this frame only (can flicker). */
    public int rawPollenCount() {
        int c = 0;
        for (Ball b : balls) if (b.isPollen) c++;
        return c;
    }

    /** Median pollen count over the last few frames (stable; use this for decisions). */
    public int getPollenCount() {
        int[] s = Arrays.copyOf(countHistory, SMOOTH_FRAMES);
        Arrays.sort(s);
        return s[SMOOTH_FRAMES / 2];
    }

    /** All pollen in this frame, closest (biggest) first. */
    public List<Ball> getPollen() {
        List<Ball> p = new ArrayList<>();
        for (Ball b : balls) if (b.isPollen) p.add(b);
        Collections.sort(p, (a, c) -> Double.compare(c.radiusPx, a.radiusPx));
        return p;
    }

    /** Closest pollen (largest radius), or null if none. Good target for driving/aiming. */
    public Ball getBestPollen() {
        List<Ball> p = getPollen();
        return p.isEmpty() ? null : p.get(0);
    }

    public boolean hasFreshData() { return fresh; }

    public void stop() { limelight.stop(); }
}
