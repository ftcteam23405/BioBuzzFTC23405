package org.firstinspires.ftc.teamcode.commandbase;

import static com.pedropathing.ivy.groups.Groups.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Latch;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.pedro.Constants;

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
    public Alliance alliance;

    private final List<LynxModule> hubs;
    private final com.pedropathing.utils.Timer loopTimer = new com.pedropathing.utils.Timer();
    private static Pose endPose = new Pose(0, 0, 0);
    public double loops = 0, lastLoop = 0, loopTime = 0;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        intake = new Intake(hardwareMap);
        latch = new Latch(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap, alliance);
        follower = Constants.create(hardwareMap);

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

        // the limelight can only run one pipeline, so claim the tags one while the shooter or turret is tracking
        if (shooter.isTracking() || turret.isTracking())
            limelight.switchToShootPipeline();

        // one read of the limelight per loop, everything below just uses what it cached
        limelight.periodic();

        // freshDistanceToGoal() is NaN unless a new tag frame came in this loop, which is what the
        // filter wants: pedro moves the estimate every loop, the limelight corrects it once per frame
        if (shooter.isTracking())
            shooter.setVelocityFromDistance(follower.pose(), alliance, limelight.freshDistanceToGoal());
        shooter.periodic();

        // same deal for the turret: pedro turns it with the robot every loop, a new tag frame's tx corrects it
        turret.aim(follower.pose(), alliance, limelight.freshAngle());
        turret.periodic();
    }

    public void setEnd() { //use at end of all autos
        endPose = follower.pose();
    }
    public Pose getEnd() {
        return endPose;
    }

    // turning in place doesn't change the distance, but the turret angle filter would read the heading
    // jump as the robot turning, so that one gets reset
    public void resetHeading() {
        follower.setPose(follower.pose().withHeading(alliance == Alliance.BLUE ? Math.toRadians(180) : 0));
        turret.resetFilter();
    }

    // call after moving the robot's position with follower.setPose(), or the filters read the jump as motion
    public void setPose(Pose pose) {
        follower.setPose(pose);
        shooter.resetFilter();
        turret.resetFilter();
    }

    public CommandBuilder intake() {
        return sequential(
                latch.toOpenPos(),
                intake.on()
        );
    }


    public double getLoopTimeMs() {
        return loopTime;
    }

    public double getLoopTimeHz() {
        return 1000 / loopTime;
    }
}
