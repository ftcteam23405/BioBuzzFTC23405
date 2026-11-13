package org.firstinspires.ftc.teamcode.opmodes.autonomous.programs;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commandbase.Robot;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pathing.Complementary2TipPath;

// path-only version of the complementary 2 tip auto, no mechanisms
@Configurable
public class Complementary2Tip extends OpMode {

    Robot robot;
    final Alliance alliance;

    private final Complementary2TipPath paths = new Complementary2TipPath();

    public Complementary2Tip(Alliance alliance) {
        this.alliance = alliance;
    }

    public Command autoRoutine() {
        return sequential(
                follow(robot.follower, paths.leave()),
                follow(robot.follower, paths.goToFlower1()),
                follow(robot.follower, paths.goAcross1()),
                follow(robot.follower, paths.goAcross2()),
                follow(robot.follower, paths.intakeGarden()),
                follow(robot.follower, paths.goToShoot1()),
                follow(robot.follower, paths.goToFlower2()),
                follow(robot.follower, paths.goToShoot2()),
                follow(robot.follower, paths.park1()),
                follow(robot.follower, paths.park2())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        robot = new Robot(hardwareMap, alliance);
        robot.setPose(PoseFactory.degrees().of(57.5, 132, 90)); // startPose in Complementary2TipPath
    }

    @Override
    public void start() {
        robot.turret.setTargetDegrees(0);
        Scheduler.schedule(autoRoutine());
    }

    @Override
    public void loop() {
        Scheduler.execute();
        robot.periodic();

        telemetry.addLine(robot.follower.debug().drivetrain().toString());
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
        robot.setEnd();
        Scheduler.reset();
    }
}
