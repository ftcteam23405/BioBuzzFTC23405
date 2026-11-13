package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.commandbase.vision.PollenDetector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

// everything is read once per loop in periodic(), getters hand back what is cached
@Configurable
@Config
public class Limelight {
    private final Limelight3A limelight;

    private Alliance alliance;
    private static final int redTags = 0, blueTags = 1, blob = 2;
    private final int tagsPipeline;

    private int currentPipeline = -1; // -1 so the first setPipleline() always goes through
    private final PollenDetector pollenDetector;

    // blob results, refreshed in periodic()
    private List<PollenDetector.Ball> balls = new ArrayList<>();
    private int pollenCount = 0;
    private PollenDetector.Ball bestPollen = null;

    // tag results, refreshed in periodic()
    private double goalDistance = Double.NaN;          // inches to the closest goal center, NaN if no tag
    private double goalAngle = Double.NaN;             // deg from the crosshair to that goal center
    private Map<Integer, Double> tagDistances = new LinkedHashMap<>();
    private Map<Integer, Double> tagAngles = new LinkedHashMap<>();
    private Map<Integer, Double> tagRawTx = new LinkedHashMap<>(); // the limelight's own tx, nothing applied
    private List<Integer> tagIds = new ArrayList<>();
    private boolean newTagFrame = false;               // true only on the loop a new limelight frame arrived
    private long lastFrameStamp = Long.MIN_VALUE;      // hub timestamp of the last frame we used
    private double lastFrameTs = Double.NaN;           // limelight-local timestamp of the same frame
    private int tagFrames = 0;                         // new frames seen, for telemetry
    private long lastGoodFrameMs = 0;                  // when we last cached a usable frame
    private long frameCaptureNanos = 0;                // System.nanoTime() when the cached frame's image was taken
    private double frameLatencyMs = Double.NaN;        // ms from image taken to us reading it, for telemetry

    public static double distanceOffset = 4.5; // inches added to the camera-to-goal distance (camera to shooter)
    // tag center height - limelight lens height (inches, hive at its tallest point), ground = sqrt(r^2 - h^2)
    public static double tagHeightAboveCamera = 36.2;
    public static long maxStalenessMs = 150; // ignore results older than this
    public static long tagTimeoutMs = 250;   // forget the last tag reading after this long with no new frame
    // everything downstream (turret aim, testers) assumes positive tx = goal is RIGHT of the crosshair, which is
    // the limelight's own convention. our camera is upside down but the limelight UI orientation is set to
    // match, so tx already comes in the right way: keep this false. only set it true if the image itself is
    // flipped (orientation not set in the UI). it flips the raw tx only, the goal-center correction comes from
    // the 3d pose and is right either way.
    // the old "turret turned away from the tag" was the dedupe bug (same tx reused every loop), not the sign
    public static boolean txReversed = false;

    //   |<-- 6.5 -->|<- 2.75 ->|<- 2.75 ->|<-- 6.5 -->|
    //   [tag]  [tag]     center     [tag]  [tag]
    //    +6.5  +2.75        0       -2.75   -6.5        <- offsets
    private static final double INNER_TAG = 2.75, OUTER_TAG = 6.5;

    // cells use tags 30-45, 4 per cell, and this assumes the ids run left to right as you face
    // if a cell's ids run the other way, negate its four numbers (or swap the argument order)
    private static final Map<Integer, Double> GOAL_OFFSETS = new HashMap<>();
    static {
        cell(30, 31, 32, 33); // cell 1, left to right
        cell(34, 35, 36, 37); // cell 2
        cell(38, 39, 40, 41); // cell 3
        cell(42, 43, 44, 45); // cell 4
    }

    // the 4 tags of one cell in the order you see them, left to right, standing in front of it
    private static void cell(int left, int midLeft, int midRight, int right) {
        GOAL_OFFSETS.put(left, OUTER_TAG);       // goal center is 6.5 in to this tag's right
        GOAL_OFFSETS.put(midLeft, INNER_TAG);    // 2.75 in to its right
        GOAL_OFFSETS.put(midRight, -INNER_TAG);  // 2.75 in to its left
        GOAL_OFFSETS.put(right, -OUTER_TAG);     // 6.5 in to its left
    }

    // live offset tuning from the dashboard: set tuneTagId to a tag, then drag tuneOffset until that tag
    // reads the same distance as the rest of its cell. -1 = off, everything comes from GOAL_OFFSETS
    // the table above is measured off the drawing, so this is only for checking it on a real field
    public static int tuneTagId = -1;
    public static double tuneOffset = 0;

    public Limelight(HardwareMap hardwareMap, Alliance a) {
        alliance = a;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pollenDetector = new PollenDetector(hardwareMap, "limelight");
        tagsPipeline = alliance == Alliance.RED ? redTags : blueTags;

        limelight.setPollRateHz(40);
        setPipleline(tagsPipeline);
        limelight.start();
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public void pause() {
        limelight.pause();
    }

    public void setPipleline(int pipleline) {
        if (pipleline == currentPipeline) return;

        limelight.pipelineSwitch(pipleline);
        currentPipeline = pipleline;
        clearTags();
        clearBlobs();
    }

    public int getPipleline() {
        return currentPipeline;
    }

    public void switchToShootPipeline() {
        setPipleline(tagsPipeline);
        if (!limelight.isRunning())
            limelight.start();
    }

    public void switchToBlobPipeline() {
        setPipleline(blob);
        if (!limelight.isRunning())
            limelight.start();
    }

    public boolean onTagPipeline() {
        return currentPipeline == tagsPipeline;
    }

    // call once per loop. reads the limelight exactly once and caches whatever the current pipeline gives us
    public void periodic() {
        newTagFrame = false;

        if (currentPipeline == blob) {
            clearTags();
            updateBlobs();
            return;
        }

        clearBlobs();
        updateTags();

        // a dropped frame or two is normal, but once the tag has been gone this long the cached
        // distance is meaningless. clearing it is what makes distanceToGoal() go back to NaN
        if (!newTagFrame && System.currentTimeMillis() - lastGoodFrameMs > tagTimeoutMs)
            clearTags();
    }

    private void updateTags() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return;                  // no detection, keep the last numbers
        if (result.getStaleness() > maxStalenessMs) return;               // too old to use
        if (result.getPipelineIndex() != currentPipeline) return;         // still switching pipelines

        // we loop faster than the limelight sends new images, so the same image comes back several times.
        // only count it as a new frame once, or the filter blends one reading in over and over, and the
        // turret subtracts the same old tx from a newer turret angle every loop (it chases itself).
        // the hub timestamp changes on every poll even for the same image, so the limelight's own ts is
        // what tells frames apart. it's 0 if the limelight didn't send one, then fall back to the hub's
        long stamp = result.getControlHubTimeStamp();
        double ts = result.getTimestamp();
        boolean repeat = ts != 0 ? equalTs(ts, lastFrameTs) : stamp == lastFrameStamp;
        if (repeat) return;

        List<Integer> ids = new ArrayList<>();
        Map<Integer, Double> distances = new LinkedHashMap<>();
        Map<Integer, Double> angles = new LinkedHashMap<>();
        Map<Integer, Double> rawTx = new LinkedHashMap<>();
        double closest = Double.NaN;
        double closestAngle = Double.NaN;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag == null) continue;

            double d = distanceToGoalCenter(tag);
            if (Double.isNaN(d)) continue;

            double a = angleToGoalCenter(tag);
            ids.add(tag.getFiducialId());
            distances.put(tag.getFiducialId(), d);
            angles.put(tag.getFiducialId(), a);
            rawTx.put(tag.getFiducialId(), tag.getTargetXDegrees());

            // the pipeline only detects our alliance's tags, so the closest tag is on one of our 2 goals
            if (Double.isNaN(closest) || d < closest) {
                closest = d;
                closestAngle = a;
            }
        }

        if (ids.isEmpty()) return; // valid result but nothing we can use, keep the last numbers

        lastFrameStamp = stamp;
        lastFrameTs = ts;

        // when the image was actually taken: exposure + processing on the limelight, plus how long it sat
        // on the hub before we read it. the turret uses this to compare tx with where it was pointing THEN
        frameLatencyMs = result.getCaptureLatency() + result.getTargetingLatency() + result.getStaleness();
        frameCaptureNanos = System.nanoTime() - (long) (frameLatencyMs * 1e6);

        tagIds = ids;
        tagDistances = distances;
        tagAngles = angles;
        tagRawTx = rawTx;
        goalDistance = closest;
        goalAngle = closestAngle;
        newTagFrame = true;
        tagFrames++;
        lastGoodFrameMs = System.currentTimeMillis();
    }

    private static boolean equalTs(double a, double b) {
        return (Double.isNaN(a) && Double.isNaN(b)) || a == b;
    }

    private void clearTags() {
        goalDistance = Double.NaN;
        goalAngle = Double.NaN;
        tagDistances = new LinkedHashMap<>();
        tagAngles = new LinkedHashMap<>();
        tagRawTx = new LinkedHashMap<>();
        tagIds = new ArrayList<>();
    }

    // ground distance to the center of the closest goal
    public double distanceToGoal() {
        return goalDistance;
    }

    // same number, but only on the loop a new limelight frame came in, NaN otherwise
    // this is what the shooter's filter wants cuz it's a reading it hasn't already used
    public double freshDistanceToGoal() {
        return newTagFrame ? goalDistance : Double.NaN;
    }

    // horizontal angle from the crosshair to the closest goal center in degrees, positive = goal is right
    // NaN if no tag has been seen. this is the tag's tx corrected for the tag's offset to the goal center
    public double angle() {
        return goalAngle;
    }

    public double freshAngle() {
        return newTagFrame ? goalAngle : Double.NaN;
    }

    // freshDistanceToGoal() / freshAngle(), but only from the tags on one cell (e.g. Robot.closestCellTags()),
    // so a tag on our other cell can't get blended into the estimate for this one. NaN if none are in view
    public double freshDistanceToGoal(int[] cellTags) {
        int id = closestTag(cellTags);
        return newTagFrame && id >= 0 ? tagDistances.get(id) : Double.NaN;
    }

    public double freshAngle(int[] cellTags) {
        int id = closestTag(cellTags);
        return newTagFrame && id >= 0 ? tagAngles.get(id) : Double.NaN;
    }

    // angle() from one cell's tags, held between frames like angle(). NaN if none of them are in view
    public double angle(int[] cellTags) {
        int id = closestTag(cellTags);
        return id >= 0 ? tagAngles.get(id) : Double.NaN;
    }

    // the raw limelight tx of the tag angle(cellTags) uses: no goal-center offset, no txReversed flip
    public double rawTx(int[] cellTags) {
        int id = closestTag(cellTags);
        return id >= 0 ? tagRawTx.get(id) : Double.NaN;
    }

    // the visible tag from cellTags that angle(cellTags) / freshAngle(cellTags) use, -1 if none
    public int closestTagId(int[] cellTags) {
        return closestTag(cellTags);
    }

    // the visible tag from cellTags with the shortest distance reading, -1 if none are visible
    private int closestTag(int[] cellTags) {
        int best = -1;
        for (int id : cellTags) {
            Double d = tagDistances.get(id);
            if (d != null && (best < 0 || d < tagDistances.get(best))) best = id;
        }
        return best;
    }

    public boolean hasTarget() {
        return !Double.isNaN(goalDistance);
    }

    // true only on the loop a new limelight frame was cached
    public boolean hasNewTagFrame() {
        return newTagFrame;
    }

    // new tag frames since startup, should climb at the limelight's real frame rate while a tag is in view
    // (never faster than the 40hz poll rate, if it matches the loop rate the dedupe is broken)
    public int getTagFrames() {
        return tagFrames;
    }

    // System.nanoTime() when the image behind the cached tag numbers was taken
    public long getFrameCaptureNanos() {
        return frameCaptureNanos;
    }

    // ms from that image being taken to us reading it, NaN before the first frame
    public double getFrameLatencyMs() {
        return frameLatencyMs;
    }

    public List<Integer> visibleTagIds() {
        return Collections.unmodifiableList(tagIds);
    }

    // distance to the goal center from every visible tag, keyed by tag id
    // tags on the same cell should all read about the same, if not fix that tag's offset
    public Map<Integer, Double> tagDistances() {
        return Collections.unmodifiableMap(tagDistances);
    }

    // horizontal distance from the camera to the center of the tag's goal, in inches
    private double distanceToGoalCenter(LLResultTypes.FiducialResult tag) {
        // where the camera is, in the tag's frame
        Position p = tag.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH);
        if (p.x == 0 && p.z == 0) return Double.NaN; // no pose solution for this tag, all zeros

        double[] toGoal = cameraToGoal(p.x, groundDepth(p), goalOffset(tag.getFiducialId()));

        return Math.hypot(toGoal[0], toGoal[1]) + distanceOffset;
    }


    // slanted, but x is level: ground depth^2 = (x^2 + y^2 + z^2) - h^2 - x^2
    private static double groundDepth(Position p) {
        double h = tagHeightAboveCamera;
        return Math.sqrt(Math.max(0, p.y * p.y + p.z * p.z - h * h));
    }

    // tx points at the tag, the goal center is off to the side of it, so add the angle between the two rays
    // positive = goal center is right of the crosshair
    private double angleToGoalCenter(LLResultTypes.FiducialResult tag) {
        double tx = txReversed ? -tag.getTargetXDegrees() : tag.getTargetXDegrees();
        double offset = goalOffset(tag.getFiducialId());
        if (offset == 0) return tx;

        Position p = tag.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH);
        double depth = groundDepth(p);
        double[] toGoal = cameraToGoal(p.x, depth, offset);
        double[] toTag = cameraToGoal(p.x, depth, 0);

        // angle from the camera->tag ray to the camera->goal ray, signed the same way tx is
        // (target space x points left as you face the tag, so the cross product gets flipped to match tx)
        double cross = toTag[1] * toGoal[0] - toTag[0] * toGoal[1];
        double dot = toTag[0] * toGoal[0] + toTag[1] * toGoal[1];

        return tx + Math.toDegrees(Math.atan2(cross, dot));
    }

    // vector from the camera to the goal center flat on the floor, as {along the tag, out from the tag}
    // camera is px along and depth out in front of the tag, the goal center sits on the tag's face at x = -offset
    private static double[] cameraToGoal(double px, double depth, double offset) {
        return new double[] {-offset - px, -depth};
    }

    // inches from the tag to its goal center, 0 for a tag that isn't in the table
    public static double goalOffset(int id) {
        if (id == tuneTagId) return tuneOffset;

        Double offset = GOAL_OFFSETS.get(id);
        return offset == null ? 0 : offset;
    }

    private void updateBlobs() {
        LLResult result = limelight.getLatestResult();

        // still switching pipelines, so the python output is from the old pipeline
        if (result == null || result.getPipelineIndex() != blob) {
            clearBlobs();
            return;
        }

        balls = pollenDetector.update();
        pollenCount = pollenDetector.getPollenCount();
        bestPollen = pollenDetector.getBestPollen();
    }

    private void clearBlobs() {
        balls = new ArrayList<>();
        pollenCount = 0;
        bestPollen = null;
    }

    // pollen visible, median over the last few frames (use this for decisions)
    public int getPollenCount() {
        return pollenCount;
    }

    // closest pollen (largest radius), or null if none
    public PollenDetector.Ball getBestPollen() {
        return bestPollen;
    }

    public boolean hasPollen() {
        return bestPollen != null;
    }

    // every ball from the latest frame, pollen and other
    public List<PollenDetector.Ball> getBalls() {
        return balls;
    }

    public boolean hasFreshBlobData() {
        return currentPipeline == blob && pollenDetector.hasFreshData();
    }

}
