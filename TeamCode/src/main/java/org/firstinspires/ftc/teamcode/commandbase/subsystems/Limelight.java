package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

// everything is read once per loop in periodic(), the getters just hand back what it cached
// so call periodic() every loop, and switchToShootPipeline() / switchToBlobPipeline() to pick what it reads
@Configurable
@Config
public class Limelight {
    private final Limelight3A limelight;

    private final Servo llServo;
    public static double tagsServoPos = 0.5;
    public static double blobServoPos = 0.5;

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
    private List<Integer> tagIds = new ArrayList<>();
    private boolean newTagFrame = false;               // true only on the loop a new limelight frame arrived
    private long lastFrameStamp = Long.MIN_VALUE;      // hub timestamp of the last frame we used
    private double lastFrameTs = Double.NaN;           // limelight-local timestamp of the same frame
    private int tagFrames = 0;                         // new frames seen, for telemetry
    private long lastGoodFrameMs = 0;                  // when we last cached a usable frame

    public static double distanceOffset = 8; // inches added to the camera-to-goal distance (camera to shooter)
    public static long maxStalenessMs = 150; // ignore results older than this
    public static long tagTimeoutMs = 250;   // forget the last tag reading after this long with no new frame

    //   |<-- 6.5 -->|<- 2.75 ->|<- 2.75 ->|<-- 6.5 -->|
    //   [tag]  [tag]     center     [tag]  [tag]
    //    +6.5  +2.75        0       -2.75   -6.5        <- offsets
    private static final double INNER_TAG = 2.75, OUTER_TAG = 6.5;

    // BIOBUZZ cells use tags 30-45, 4 per cell, and this assumes the ids run left to right as you face
    // the cell. if a cell's ids run the other way, negate its four numbers (or swap the argument order)
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
        llServo = hardwareMap.get(Servo.class, "llServo");
        pollenDetector = new PollenDetector(hardwareMap, "limelight");
        tagsPipeline = alliance == Alliance.RED ? redTags : blueTags;

        limelight.setPollRateHz(20);
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
        llServo.setPosition(tagsServoPos);
        if (!limelight.isRunning())
            limelight.start();
    }

    public void switchToBlobPipeline() {
        setPipleline(blob);
        llServo.setPosition(blobServoPos);
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

        // the limelight polls at 20hz and we loop faster than that, so the same result comes back several
        // times. only count it as a new frame once, or the filter blends one reading in over and over
        long stamp = result.getControlHubTimeStamp();
        double ts = result.getTimestamp();
        if (stamp == lastFrameStamp && equalTs(ts, lastFrameTs)) return;
        lastFrameStamp = stamp;
        lastFrameTs = ts;

        List<Integer> ids = new ArrayList<>();
        Map<Integer, Double> distances = new LinkedHashMap<>();
        double closest = Double.NaN;
        double closestAngle = Double.NaN;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag == null) continue;

            double d = distanceToGoalCenter(tag);
            if (Double.isNaN(d)) continue;

            ids.add(tag.getFiducialId());
            distances.put(tag.getFiducialId(), d);

            // the pipeline only detects our alliance's tags, so the closest tag is on the closest goal
            if (Double.isNaN(closest) || d < closest) {
                closest = d;
                closestAngle = angleToGoalCenter(tag);
            }
        }

        if (ids.isEmpty()) return; // valid result but nothing we can use, keep the last numbers

        tagIds = ids;
        tagDistances = distances;
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
        tagIds = new ArrayList<>();
    }

    // straight-line distance to the center of the closest goal in inches, or NaN if no tag has been seen
    // every tag is shifted to its goal center first, so it doesn't matter which tag is detected
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

    public boolean hasTarget() {
        return !Double.isNaN(goalDistance);
    }

    // true only on the loop a new limelight frame was cached
    public boolean hasNewTagFrame() {
        return newTagFrame;
    }

    // new tag frames since startup, should climb at about 20hz while a tag is in view
    public int getTagFrames() {
        return tagFrames;
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

        // x and z are the flat plane through the tag, y is height, so this ignores how high the tag is
        double[] xz = cameraToGoal(p.x, p.z, goalOffset(tag.getFiducialId()));

        return Math.hypot(xz[0], xz[1]) + distanceOffset;
    }


    // tx points at the tag, the goal center is off to the side of it, so add the angle between the two rays
    private double angleToGoalCenter(LLResultTypes.FiducialResult tag) {
        double offset = goalOffset(tag.getFiducialId());
        if (offset == 0) return tag.getTargetXDegrees();

        Position p = tag.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH);
        double[] toGoal = cameraToGoal(p.x, p.z, offset);
        double[] toTag = cameraToGoal(p.x, p.z, 0);

        // angle from the camera->tag ray to the camera->goal ray, signed the same way tx is
        // (target space x points left as you face the tag, so the cross product gets flipped to match tx)
        double cross = toTag[1] * toGoal[0] - toTag[0] * toGoal[1];
        double dot = toTag[0] * toGoal[0] + toTag[1] * toGoal[1];

        return tag.getTargetXDegrees() + Math.toDegrees(Math.atan2(cross, dot));
    }

    // vector from the camera to the goal center, in the tag's flat plane, as {x, z}
    // camera is at (px, pz) in target space and the goal center sits on the tag's face at x = -offset
    private static double[] cameraToGoal(double px, double pz, double offset) {
        return new double[] {-offset - px, -pz};
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
