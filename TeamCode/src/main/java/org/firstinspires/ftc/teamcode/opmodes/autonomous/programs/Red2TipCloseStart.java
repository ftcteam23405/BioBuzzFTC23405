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
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commandbase.Robot;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;

@Configurable
@Autonomous
public class Red2TipCloseStart extends OpMode {

    Robot robot;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(54, 9, 90);
    private final Pose intakeEnd = poseFactory.of(11.56, 11.05, 240);
    private final Pose intakeCtrl1 = poseFactory.of(40.26, 27.29, 0);
    private final Pose goAcross1End = poseFactory.of(58, 26, 90);
    private final Pose goAcross2End = poseFactory.of(58, 117, 90);
    private final Pose goToFlowerEnd = poseFactory.of(47, 123, 90);
    private final Pose parkEnd = poseFactory.of(16, 117, 0);

    public Path intake() {
        return curve(startPose, intakeCtrl1, intakeEnd).linear(startPose, intakeEnd);
    }

    public Path goAcross1() { return line(intakeEnd, goAcross1End).linear(intakeEnd, goAcross1End); }
    public Path goAcross2() { return line(goAcross1End, goAcross2End).constant(90); }
    public Path goAcross() { //concatenation of goAcross paths
        return Paths.path(goAcross1(), goAcross2());
    }

    public Path goToFlower() {
        return line(goAcross2End, goToFlowerEnd).linear(goAcross2End, goToFlowerEnd);
    }

    public Path park() {
        return line(goToFlowerEnd, parkEnd).linear(goToFlowerEnd, parkEnd);
    }

    public Command autoRoutine() {
        return sequential(
                follow(robot.follower, intake()),
                follow(robot.follower, goAcross()),
                follow(robot.follower, goToFlower()),
                follow(robot.follower, park())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        robot = new Robot(hardwareMap, Alliance.RED);
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
        robot.setEnd(); // so teleop starts from where auto left off and the turret aims from the right spot
        Scheduler.reset();
    }

}
