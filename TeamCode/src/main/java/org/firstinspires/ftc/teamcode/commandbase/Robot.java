package org.firstinspires.ftc.teamcode.commandbase;

import static com.pedropathing.ivy.groups.Groups.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Latch;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;
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
    public final DriveTrain driveTrain;
    public final Follower follower;
    public Alliance alliance;

    public final List<LynxModule> hubs;
    private final com.pedropathing.utils.Timer loopTimer = new com.pedropathing.utils.Timer();
    private static Pose endPose = new Pose(0, 0, 0);
    public double loops = 0, lastLoop = 0, loopTime = 0;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        intake = new Intake(hardwareMap);
        latch = new Latch(hardwareMap);
        shooter = new Shooter(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap);

        follower = Constants.create(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        loopTimer.reset();

    }

    public void periodic(OpMode teleop) {
        shooter.periodic(teleop);
        follower.update();
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
    }

    // call after moving the robot's position with follower.setPose(), or the filters read the jump as motion
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public CommandBuilder intake() {
        return sequential(
                latch.doIntake(),
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
