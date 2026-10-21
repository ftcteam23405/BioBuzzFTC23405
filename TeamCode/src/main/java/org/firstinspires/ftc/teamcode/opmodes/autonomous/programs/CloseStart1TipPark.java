package org.firstinspires.ftc.teamcode.opmodes.autonomous.programs;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commandbase.Robot;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;

@Configurable
public class CloseStart1TipPark extends OpMode{

    Robot robot;
    final Alliance alliance;

    private final Pose startPose;
    private final Pose shoot1End;
    private final Pose parkPose;
    private final Pose parkCtrl1;

    public CloseStart1TipPark(Alliance alliance) {
        this.alliance = alliance;

        PoseFactory poseFactory = (alliance == Alliance.RED) ? PoseFactory.degrees() : PoseFactory.degrees().mirrorAroundPoint(71,71);
        startPose = poseFactory.of(56, 9, 90);
        shoot1End = poseFactory.of(56, 27, 90);
        parkPose = poseFactory.of(13.81, 93.04, 0);
        parkCtrl1 = poseFactory.of(20.02, 32.89, 0);
    }

    public Path goToShoot() {
        return line(startPose, shoot1End).constant(90);
    }

    public Path park() {
        return curve(shoot1End, parkCtrl1, parkPose).linear(shoot1End, parkPose);
    }

    public Command autoRoutine() {
        return sequential(
                parallel(follow(robot.follower, goToShoot()), robot.shooter.autoPower()),
                shootSequence(),
                follow(robot.follower, park())
        );
    }

    public Command shootSequence() {
        return sequential(
                waitMs(1000),
                parallel(robot.latch.doIntake(), robot.intake.on()),
                waitMs(200),
                robot.shooter.autoPower(),
                waitMs(2000),
                parallel(robot.shooter.off(), robot.intake.off(), robot.latch.off())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        robot = new Robot(hardwareMap, alliance);
        robot.setPose(startPose);
    }

    @Override
    public void start() {
        Scheduler.schedule(autoRoutine());
    }

    @Override
    public void loop() {
        Scheduler.execute();
        robot.periodic();
        String drivetrainString = robot.follower.debug().drivetrain().toString();
        telemetry.addLine(drivetrainString);
        telemetry.addData("X", robot.follower.pose().x());
        telemetry.addData("Y", robot.follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(robot.follower.pose().heading()));
        telemetry.addData("Follower Mode", robot.follower.mode());
        telemetry.addData("Path completion", robot.follower.completion());
        telemetry.addData("Distance remaining in path", robot.follower.remainingDistance());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.setEnd(); // so teleop starts from where auto left off
        Scheduler.reset();
    }
}

