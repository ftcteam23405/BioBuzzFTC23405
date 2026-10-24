package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.commands.Commands.waitMs;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.commandbase.Robot;

@Config
@Configurable

public class Teleop extends OpMode {

    JoinedTelemetry joinedTelemetry;

    Robot robot;
    final Alliance alliance;
    public boolean hold = false;

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
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        Scheduler.execute();

        DrivePowers powers = ManualDrive.fieldCentric(
            -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, robot.follower.pose().heading(), (robot.alliance == Alliance.BLUE ? Math.toRadians(180) : Math.toRadians(0))
        );

        robot.follower.manual(powers);
        if (gamepad1.startWasPressed())
            robot.resetHeading();

        if (gamepad1.rightBumperWasPressed()) {
            robot.shooter.enable();
        }
        if (gamepad1.rightBumperWasReleased()) {
            robot.shooter.stop();
        }
        if (gamepad1.right_trigger > 0.5) {
            robot.intake.intakeOn();
        }
        else{
            robot.intake.intakeOff();
        }
        if (gamepad1.left_trigger_pressed) {
            robot.transfer.intake();
        }
        else {
            robot.transfer.stop();
        }
        if (gamepad1.xWasPressed())
            robot.intake.off();
        if (gamepad1.bWasPressed())
            robot.transfer.stop();



        if (gamepad1.yWasPressed())
            robot.intake.intakeOn();
        if (gamepad1.aWasPressed()){
            Scheduler.schedule(robot.shooterSeq());

        }


        robot.periodic();
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
        joinedTelemetry.addData("Intake", robot.intake.getCurrent());
        joinedTelemetry.addData("Shooter", robot.shooter.getCurrent());
        joinedTelemetry.addData("Drivetrain", robot.driveTrain.getCurrent());
        joinedTelemetry.addData("speed",robot.shooter.getVelocity());

        for (LynxModule hub : robot.hubs) {
            joinedTelemetry.addData(hub.getDeviceName(), hub.getCurrent(CurrentUnit.MILLIAMPS)+", "+hub.getInputVoltage(VoltageUnit.MILLIVOLTS));
        }
        joinedTelemetry.addData("Hold Position", hold);
        joinedTelemetry.update();
    }
}
