package org.firstinspires.ftc.teamcode.commandbase;

import static com.pedropathing.ivy.groups.Groups.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Latch;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Light;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

//class where all robot mechanisms and routines are handled
// Structure: Subsystems & Commands -> Robot Class -> Teleop
public class Robot {
    public final Intake intake;
    public final Latch latch;
    public final Shooter shooter;
    public final Turret turret;
    public final Limelight limelight;
    public final Follower follower;
    public final Light light;
    public final FlowerIntake flowerMech;
    public Alliance alliance;

    private final List<LynxModule> hubs;
    private final com.pedropathing.utils.Timer loopTimer = new com.pedropathing.utils.Timer();
    private static Pose endPose = new Pose(72, 72, 0);
    private Pose goal = null; //alliance goal the shooter and turret are aiming at, picked every loop
    public double loops = 0, lastLoop = 0, loopTime = 0;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        intake = new Intake(hardwareMap);
        latch = new Latch(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap, alliance);
        follower = Constants.create(hardwareMap);
        light = new Light(hardwareMap);
        flowerMech = new FlowerIntake(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        loopTimer.reset();

        periodic();
    }

    public void periodic() {
        loops++;

        if (loops > 10) { //for ref, 50hz is a solid loop time
            double now = loopTimer.get(TimeUnit.MILLISECONDS);
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }

        follower.update();

//        if (!shooter.atTarget()) {
//            light.notAtRPM();
//        } else {
//            light.toAtTargetColor();
//        }

        if (!intake.isFull()) {
            light.intakeNotFull();
        } else {
            light.toIntakeFullColor();
        }

        //use tags pipeluine while the shooter or turret is tracking
        if (shooter.isTracking() || turret.isTracking())
            limelight.switchToShootPipeline();

        //everything below uses what ll just cached
        limelight.periodic();

        // one goal, picked once per loop so the shooter and turret always agree. only that goal's tags go to
        // the filters. the limelight picks it when one of our cells is in view (only one can be at a time), so
        // a wrong pedro pose can't make the filters ignore the tags. pedro's pick is only the fallback with no
        // tag in view. ODOMETRY (pedro-only testing) always uses pedro's pick
        Pose pose = follower.pose();
        Pose seen = turret.getAimSource() != Turret.AimSource.ODOMETRY ? visibleGoal() : null;
        goal = seen != null ? seen : Shooter.closestGoal(pose, goal);
        int[] goalTags = cellTags(goal);


        // filters want: pedro moves the estimate every loop, the limelight corrects it once per frame
        if (shooter.isTracking())
            shooter.setVelocityFromDistance(pose, goal, limelight.freshDistanceToGoal(goalTags));
        shooter.periodic();


        turret.aim(pose, goal, limelight.freshAngle(goalTags), limelight.getFrameCaptureNanos());
        turret.periodic();

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    public void setEnd() { //use at end of all autos
        endPose = follower.pose();
    }
    public Pose getEnd() {
        return endPose;
    }

    // turning in place doesn't change the distance, but the turret angle filter would read the heading
    // jump as the robot turning, so that one gets reset. 0 for both alliances, blue's frame is red's turned 180
    public void resetHeading() {
        follower.setPose(follower.pose().withHeading(0));
        turret.resetFilter();
    }

    // call after moving the robot's position with follower.setPose(), or the filters read the jump as motion
    public void setPose(Pose pose) {
        follower.setPose(pose);
        goal = null; // pick the closest goal fresh from the new pose
        shooter.resetFilter();
        turret.resetFilter();
    }

    //audience cells are on the same end of the field for both alliances
    private static final int[] RED_FAR_TAGS = {30, 31, 32, 33};
    private static final int[] RED_AUDIENCE_TAGS = {34, 35, 36, 37};
    private static final int[] BLUE_AUDIENCE_TAGS = {38, 39, 40, 41};
    private static final int[] BLUE_FAR_TAGS = {42, 43, 44, 45};

    // the goal the shooter and turret are aiming at this loop
    public Pose getGoal() {
        return goal != null ? goal : Shooter.closestGoal(follower.pose());
    }

    // whichever of our 2 cells has a tag in view, null if none of our tags are. both cells can't be in
    // frame at once, so there's nothing to choose between
    private Pose visibleGoal() {
        Pose audience = new Pose(Shooter.goalX, Shooter.audienceGoalY);
        if (anyVisible(cellTags(audience))) return audience;
        Pose far = new Pose(Shooter.goalX, Shooter.farGoalY);
        if (anyVisible(cellTags(far))) return far;
        return null;
    }

    private boolean anyVisible(int[] cell) {
        List<Integer> visible = limelight.visibleTagIds();
        for (int id : cell) {
            if (visible.contains(id)) return true;
        }
        return false;
    }

    // the ids from closestCellTags() the limelight can see right now. these are the only tags aiming uses
    public List<Integer> acceptedTagIds() {
        List<Integer> visible = limelight.visibleTagIds();
        List<Integer> accepted = new ArrayList<>();
        for (int id : closestCellTags()) {
            if (visible.contains(id)) accepted.add(id);
        }
        return accepted;
    }

    // tag ids on our alliance's cell that's closest to the robot (the one we're aiming at)
    public int[] closestCellTags() {
        return cellTags(goal != null ? goal : Shooter.closestGoal(follower.pose()));
    }

    // tag ids on one of our alliance's cells, from Shooter.closestGoal()
    // blue runs red's coordinates with the field turned 180 deg, so for blue the ends swap: the goal at
    // audienceGoalY is physically blue's far cell, and the one at farGoalY is blue's audience cell
    private int[] cellTags(Pose goal) {
        boolean atAudienceY = goal.y() == Shooter.audienceGoalY;

        if (alliance == Alliance.RED)
            return atAudienceY ? RED_AUDIENCE_TAGS : RED_FAR_TAGS;
        return atAudienceY ? BLUE_FAR_TAGS : BLUE_AUDIENCE_TAGS;
    }

    // true if the limelight can see at least one tag on the closest cell
    public boolean closestCellTagsVisible() {
        return anyVisible(closestCellTags());
    }

    // the hive has tipped if the closest cell's tags are gone. only means anything while the limelight
    // is on the tags pipeline and pointed at the goal (turret tracking), so it's false otherwise
    public boolean hiveTipped() {
        if (!limelight.onTagPipeline() || !turret.isTracking()) return false;
        return !closestCellTagsVisible();
    }

    public void intake() {
        latch.toClosePos();
        intake.intakeOn();
    }


    public double getLoopTimeMs() {
        return loopTime;
    }

    public double getLoopTimeHz() {
        return 1000 / loopTime;
    }
}
