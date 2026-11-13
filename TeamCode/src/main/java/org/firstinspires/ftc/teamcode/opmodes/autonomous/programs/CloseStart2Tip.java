package org.firstinspires.ftc.teamcode.opmodes.autonomous.programs;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commandbase.Robot;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;

@Configurable
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


    public static double launch1Deg = -90;
    public static double launch2Deg = -185;
    public static double launch3Deg = -165;

    public static double launchMs = 2000;
    public static double intakeMs = 1200;
    public static double aimTimeoutMs = 1500;


    public CloseStart2Tip(Alliance alliance) {
        this.alliance = alliance;

        PoseFactory poseFactory = PoseFactory.degrees();

        startPose = poseFactory.of(56.5, 7.5, 180);
        intakeEnd = poseFactory.of(12, 9, 190);
        intakeCtrl1 = poseFactory.of(35.96, 14.5, 0);
        goAcross1End = poseFactory.of(63.4, 25.59, 90);
        goAcross2End = poseFactory.of(63, 116, 90);
        goToFlowerEnd = poseFactory.of(47, 123, 90);
        parkEnd = poseFactory.of(9.77, 116.36, 0);
    }

    public Path intake() {
        return curve(startPose, intakeCtrl1, intakeEnd).linear(startPose, intakeEnd);
    }

    public Path goAcross1() { return line(intakeEnd, goAcross1End).linear(intakeEnd, goAcross1End); }
    public Path goAcross2() { return line(goAcross1End, goAcross2End).constant(goAcross1End); }
    public Path goAcross() { //concatenation of goAcross paths
        return Paths.path(goAcross1(), goAcross2());
    }

    public Path goToFlower() {
        return line(goAcross2End, goToFlowerEnd).linear(goAcross2End, goToFlowerEnd);
    }

    public Path park() {
        return line(goToFlowerEnd, parkEnd).linear(goToFlowerEnd, parkEnd);
    }


    private Command launch() {
        return sequential(
                race(
                        Commands.waitUntil(() -> robot.shooter.atTarget() && robot.turret.atTarget()),
                        Commands.waitMs(aimTimeoutMs)
                ),
                Commands.instant(() -> {
                    robot.latch.open();
                    robot.intake.intakeOn();
                }),
                Commands.waitMs(launchMs),
                Commands.instant(() -> {
                    robot.intake.intakeOff();
                    robot.latch.close();
                })
        );
    }


    private Command finishIntake() {
        return sequential(
                race(
                        Commands.waitUntil(robot.intake::isFull),
                        Commands.waitMs(intakeMs)
                ),
                Commands.instant(robot.intake::intakeOff)
        );
    }

    private Command startIntake() {
        return Commands.instant(robot::intake); //closes the latch + intake on
    }


    public Command autoRoutine() {
        return sequential(
                robot.flowerMech.goUp(),
                launch(),

                startIntake(),

                parallel(follow(robot.follower, intake()), robot.turret.toTargetDegrees(launch2Deg)),
                finishIntake(),

                follow(robot.follower, goAcross()),
                launch(),

                robot.turret.toTargetDegrees(launch3Deg),
                robot.flowerMech.goDown(),
                Commands.waitMs(1000),
                startIntake(),
                follow(robot.follower, goToFlower()),
                finishIntake(),
                launch(),

                robot.turret.toTargetDegrees(0),
                Commands.instant(robot.shooter::stop),
                parallel(follow(robot.follower, park()), sequential(Commands.waitMs(300), robot.flowerMech.goUp()))

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
        robot.shooter.startTracking();
        robot.turret.setTargetDegrees(launch1Deg);
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
        telemetry.addData("Goal Distance (in)", robot.shooter.getGoalDistance());
        telemetry.addData("Shooter Target (ticks/s)", robot.shooter.getTargetVelocity());
        telemetry.addData("Shooter Velocity (ticks/s)", robot.shooter.getVelocity());
        telemetry.addData("Turret Target (deg)", robot.turret.getTargetDegrees());
        telemetry.addData("Turret Angle (deg)", robot.turret.getAngle());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.setEnd();
        Scheduler.reset();
    }

}
