package org.firstinspires.ftc.teamcode.opmodes.autonomous.programs;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commandbase.Robot;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;

@Configurable
public class CloseStart1Tip extends OpMode {

    Robot robot;
    final Alliance alliance;

    private final Pose startPose;
    private final Pose intakeEnd;
    private final Pose intakeCtrl1;

    public static double transferSpeed = 1;

    public static double launch1Deg = -90;

    public static double launchVel = 1420;

    public static double launchMs = 5000;
    public static double aimTimeoutMs = 1500;


    public CloseStart1Tip(Alliance alliance) {
        this.alliance = alliance;

        PoseFactory poseFactory = PoseFactory.degrees();

        startPose = poseFactory.of(56.5, 7.5, 180);
        intakeEnd = poseFactory.of(10.14, 96.89, 0);
        intakeCtrl1 = poseFactory.of(31.66, 20.45, 0);
    }

    public Path park() {
        return curve(startPose, intakeCtrl1, intakeEnd).linear(startPose, intakeEnd);
    }


    private Command launch() {
        return sequential(
                race(
                        Commands.waitUntil(() -> robot.shooter.atTarget() && robot.turret.atTarget()),
                        Commands.waitMs(aimTimeoutMs)
                ),
                Commands.instant(() -> {
                    robot.latch.open();
                    robot.intake.setPower(transferSpeed);
                }),
                Commands.waitMs(launchMs),
                Commands.instant(() -> {
                    robot.intake.intakeOff();
                    robot.latch.close();
                })
        );
    }


    public Command autoRoutine() {
        return sequential(
                robot.flowerMech.goUp(),
                launch(),

                robot.turret.toTargetDegrees(0),
                Commands.instant(robot.shooter::stop),
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
        robot.shooter.setTargetVelocity(launchVel);
        robot.turret.setTargetDegrees(launch1Deg);
        Scheduler.schedule(autoRoutine());
    }

    @Override
    public void loop() {
        Scheduler.execute();
        robot.periodic();
        telemetry.addData("X", robot.follower.pose().x());
        telemetry.addData("Y", robot.follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(robot.follower.pose().heading()));
        telemetry.addData("Follower Mode", robot.follower.mode());
        telemetry.addData("Path completion", robot.follower.completion());
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
