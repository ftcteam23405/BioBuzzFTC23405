package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.commandbase.Robot;

@Config
@Configurable
//one single Teleop Class that handles all Teleop Commands, then added alliance through constructor
public class Teleop extends OpMode {

    JoinedTelemetry joinedTelemetry;

    Robot robot;
    final Alliance alliance;

    public static double shooterVelocity = 1400; // ticks/s while right bumper is held, change it live in panels

    public boolean hold = false;
    public double speed = 1;

    public boolean killSwitchToggle = false;

    public double hoodPos = 0.5;

    public Teleop(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, alliance);
        robot.setPose(robot.getEnd()); //also resets the shooter's distance filter

        joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
    }

    @Override
    public void init_loop() { //what happens when looping on init

        joinedTelemetry.addData("Robot Saved Pose", robot.getEnd()); //way to check saved pose after auto
        joinedTelemetry.update();
        robot.turret.setAimSource(Turret.AimSource.ODOMETRY);
    }

    @Override
    public void start() {
        robot.turret.startTracking(); //turret faces the closest goal for the whole match

        robot.periodic();
    }

    @Override
    public void loop() {
        robot.periodic();
        Scheduler.execute();
        //if the robot is not holding position, run the Teleop Drive
        //no heading offset for either alliance: blue's coordinates are red's turned 180 deg, so the drivers line up the same
        if (!hold) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, robot.follower.pose().heading()
            );
            robot.follower.manual(powers);
        }

        if (gamepad1.startWasPressed())
            robot.resetHeading();

        if (gamepad1.right_trigger_pressed) {
            robot.intake();
        }
        else if (gamepad1.left_trigger_pressed) {
            robot.intake.intakeReverse();
        }
        else {
            robot.intake.intakeOff();
        }

        if (gamepad1.right_bumper) {
            robot.shooter.setTargetVelocity(shooterVelocity);
        }

        if (robot.shooter.atTarget()) {
            robot.latch.open();
        }

        else if (gamepad1.rightBumperWasReleased()) {
            robot.shooter.stop();
            robot.latch.close();
        }
        if(gamepad1.aWasPressed()) {
            robot.latch.toggle();
        }
        if(gamepad1.xWasPressed()) {
            robot.flowerMech.toggle();
        }


        updateTelemetry();

    }

    @Override
    public void stop() { //what happens when stop button is clicked
        robot.setEnd();
        Scheduler.reset();
    }

    public void updateTelemetry() {
        joinedTelemetry.addData("LoopTime Hz", robot.getLoopTimeHz());
        joinedTelemetry.addLine();
        joinedTelemetry.addData("Follower Pose", robot.follower.pose().toString());
        joinedTelemetry.addLine();

        joinedTelemetry.addData("Turret Position (deg)", robot.turret.getAngle());
        joinedTelemetry.addData("Turret Target (deg)", robot.turret.getTargetDegrees());
        joinedTelemetry.addData("Turret On Goal", robot.turret.atTarget());
        joinedTelemetry.addLine();

        joinedTelemetry.addData("Shooter Target (ticks/s)", robot.shooter.getTargetVelocity());
        joinedTelemetry.addData("Shooter Velocity (ticks/s)", robot.shooter.getVelocity());
        joinedTelemetry.addData("Shooter At Target", robot.shooter.atTarget());
        joinedTelemetry.addLine();

        joinedTelemetry.addData("Latch Closed", robot.latch.isOpen());
        joinedTelemetry.addData("Hold Position", hold);
        joinedTelemetry.update();
    }
}
