package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.commandbase.Robot;

@Config
@Configurable
//one single Teleop Class that handles all Teleop Commands, then added alliance through constructor
public class Teleop extends OpMode {

    JoinedTelemetry joinedTelemetry;

    Robot robot;
    final Alliance alliance;

    public static double shooterVelocity = 1300; // ticks/s while right bumper is held, change it live in panels

    public boolean hold = false;
    public double speed = 1;

    public Teleop(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() { //what happens at initialization
        robot = new Robot(hardwareMap, alliance);
        robot.setPose(robot.getEnd()); //also resets the shooter's distance filter

        joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
    }

    @Override
    public void init_loop() { //what happens when looping on init

        joinedTelemetry.addData("Robot Saved Pose", robot.getEnd()); //way to check saved pose after auto
        joinedTelemetry.update();
    }

    @Override
    public void start() { //what happens when start is pressed
        robot.turret.startTracking(); //turret faces the closest goal for the whole match
        robot.periodic();
    }

    @Override
    public void loop() { //what happens during the whole match - what runs in the background
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

        // aWasPressed() clears itself when read, so only read it once per loop
        boolean aPressed = gamepad1.aWasPressed();

        // set every loop so changing shooterVelocity in panels takes effect while holding
        if (gamepad1.right_bumper)
            robot.shooter.setTargetVelocity(shooterVelocity);
        else if (gamepad1.rightBumperWasReleased())
            robot.shooter.stop();

        if (gamepad1.xWasPressed())
            robot.latch.toOpenPos();
        if (aPressed)
            robot.latch.toClosePos();


        if (gamepad1.left_bumper)
            speed = 0.5;
        else
            speed = 1.0;

        if (gamepad1.yWasPressed())
            robot.intake.intakeOn();
        if (aPressed)
            robot.intake.intakeOff();

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
