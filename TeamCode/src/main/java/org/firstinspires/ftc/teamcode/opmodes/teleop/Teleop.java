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
        //if the alliance is Blue, then set the offset heading to PI rad. If not, then keep it 0 (for red)
        if (!hold) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, robot.follower.pose().heading(), (robot.alliance == Alliance.BLUE ? Math.toRadians(0) : Math.toRadians(180))
            );
            robot.follower.manual(powers);
        }

        if (gamepad1.startWasPressed())
            robot.resetHeading();



        if (gamepad1.xWasPressed())
            robot.latch.toOpenPos();
        if (gamepad1.aWasPressed())
            robot.latch.toClosePos();


        if (gamepad1.left_bumper)
            speed = 0.5;
        else
            speed = 1.0;

        if (gamepad1.yWasPressed())
            robot.intake.intakeOn();
        if (gamepad1.aWasPressed())
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

        joinedTelemetry.addData("Turret Angle", robot.turret.getAngle());
        joinedTelemetry.addData("Turret Target", robot.turret.getTargetAngle());
        joinedTelemetry.addData("Turret On Goal", robot.turret.atTarget());
        joinedTelemetry.addLine();

        joinedTelemetry.addData("Latch Closed", robot.latch.isOpen());
        joinedTelemetry.addData("Hold Position", hold);
        joinedTelemetry.update();
    }
}
