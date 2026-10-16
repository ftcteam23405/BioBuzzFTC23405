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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commandbase.Robot;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;

@Configurable
//one single 2 Tip Close Start auto, then added alliance through constructor (same as Teleop)
//poses are written for RED, and mirrored across the middle of the field for BLUE
public class CloseStart2Tip extends OpMode {

    Robot robot;
    final Alliance alliance;

    private final Pose startPose;
    private final Pose intakeEnd;
    private final Pose intakeCtrl1;
    private final Pose goAcross1End;
    private final Pose goAcross2End;
    private final Pose goToFlowerEnd;
    private final Pose parkEnd;

    public CloseStart2Tip(Alliance alliance) {
        this.alliance = alliance;

        PoseFactory poseFactory = (alliance == Alliance.RED) ? PoseFactory.degrees() : PoseFactory.degrees().mirrorAroundPoint(71,71);

        startPose = poseFactory.of(54, 9, 90);
        intakeEnd = poseFactory.of(11.56, 11.05, 240);
        intakeCtrl1 = poseFactory.of(40.26, 27.29, 0);
        goAcross1End = poseFactory.of(58, 26, 90);
        goAcross2End = poseFactory.of(58, 117, 90);
        goToFlowerEnd = poseFactory.of(47, 123, 90);
        parkEnd = poseFactory.of(16, 117, 0);
    }

    public Path intake() {
        return curve(startPose, intakeCtrl1, intakeEnd).linear(startPose, intakeEnd);
    }

    public Path goAcross1() { return line(intakeEnd, goAcross1End).linear(intakeEnd, goAcross1End); }
    public Path goAcross2() { return line(goAcross1End, goAcross2End).constant(goAcross1End); } //constant(double) is radians, so use the pose's heading so it mirrors too
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
        robot.setEnd(); // so teleop starts from where auto left off and the turret aims from the right spot
        Scheduler.reset();
    }

}
