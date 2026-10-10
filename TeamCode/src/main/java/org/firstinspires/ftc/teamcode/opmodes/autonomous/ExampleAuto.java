package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commandbase.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
@Configurable
public class ExampleAuto extends OpMode {

    Robot robot;

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(24,24,0);
    private final Pose controlPose = poseFactory.of(30,60,45);
    private final Pose parkPose = poseFactory.of(48, 48, 90);

    private Path park() {
        return line(startPose, parkPose).linear(startPose,parkPose);
    }

    private Path parkBezier() {
        return curve(startPose, controlPose, parkPose).linear(startPose, parkPose);
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }


    @Override
    public void start() {
        schedule(follow(follower, parkBezier()));

    }


    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        String drivetrainString = follower.debug().drivetrain().toString();
        telemetry.addLine(drivetrainString);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Path completion", follower.completion());
        telemetry.addData("Distance remaining in path", follower.remainingDistance());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.setEnd();
    }
}
