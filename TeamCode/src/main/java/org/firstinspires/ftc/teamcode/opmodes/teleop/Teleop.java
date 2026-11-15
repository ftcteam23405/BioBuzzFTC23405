package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.commandbase.Robot;

@Config
@Configurable

public class Teleop extends OpMode {

    JoinedTelemetry joinedTelemetry;

    Robot robot;
    final Alliance alliance;

    public static double shooterVelocity = 1400; // ticks/s while right bumper is held, change it live in panels

    // gamepad2 dpad up/down manual turret: one step on press, then another every turretRepeatSeconds while held
    public static double turretStepDegrees = 5;
    public static double turretRepeatSeconds = 0.15;
    private final ElapsedTime turretStepTimer = new ElapsedTime();

    public boolean hold = false;
    public double speed = 1;



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

        if (gamepad1.backWasPressed()) //skip the saved auto pose and start from the field center
            robot.setPose(new Pose(72, 72, 0));

        joinedTelemetry.addData("Robot Saved Pose", robot.getEnd()); //way to check saved pose after auto
        joinedTelemetry.addData("Follower Pose", robot.follower.pose().toString());
        joinedTelemetry.addLine("Gamepad 1 back: reset pose to (72, 72, 0)");
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

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, robot.follower.pose().heading()
        );
        robot.follower.manual(powers);


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

        if (gamepad1.rightBumperWasReleased()) {
            robot.latch.close();
            robot.intake.intakeOff();
        }

        if (gamepad1.right_bumper) {
            robot.latch.open();
        }

        if (gamepad1.yWasPressed()) {
            robot.flowerMech.up();
        }

        if (gamepad1.aWasPressed()) {
            robot.flowerMech.down();
        }


        if (gamepad2.right_trigger_pressed) {
            robot.shooter.setVelocityFromDistance(robot.follower.pose());
        }

        if (gamepad2.left_trigger_pressed) {
            robot.shooter.setTargetVelocity(shooterVelocity);
        }

        if (gamepad2.bWasPressed()) {
            robot.shooter.setTargetVelocity(0);
        }

        if (gamepad2.yWasPressed()) {
            robot.flowerMech.toggle();
        }

        if (gamepad2.startWasPressed()) {
            robot.relocalizeCorner();
        }

        if (gamepad2.backWasPressed()) {
            robot.toggleTurretKill();
        }

        manualTurret();


        updateTelemetry();

    }

    @Override
    public void stop() { //what happens when stop button is clicked
        robot.setEnd();
        Scheduler.reset();
    }

    // dpad up = +5 deg (left / ccw), dpad down = -5 deg (right / cw). steps from the current target so held
    // steps add up, and setTarget clips to the turret limits. stepping stops goal tracking, x goes back to it
    private void manualTurret() {
        double direction = 0;
        if (gamepad2.dpad_up) direction += 1;
        if (gamepad2.dpad_down) direction -= 1;

        // read both every loop (no ||) so neither press flag is left latched for a later loop
        boolean upPressed = gamepad2.dpadUpWasPressed();
        boolean downPressed = gamepad2.dpadDownWasPressed();
        boolean pressed = upPressed || downPressed;
        if (direction != 0 && (pressed || turretStepTimer.seconds() >= turretRepeatSeconds)) {
            robot.turret.setTargetDegrees(robot.turret.getTargetDegrees() + direction * turretStepDegrees);
            turretStepTimer.reset();
        }

        if (gamepad2.xWasPressed()) {
            robot.turret.startTracking();
        }
    }

    public void updateTelemetry() {
        joinedTelemetry.addData("LoopTime Hz", robot.getLoopTimeHz());
        joinedTelemetry.addLine();
        joinedTelemetry.addData("Follower Pose", robot.follower.pose().toString());
        joinedTelemetry.addLine();

        joinedTelemetry.addData("Intake full", robot.intake.isFull());
        joinedTelemetry.addLine();

        joinedTelemetry.addData("Turret Position (deg)", robot.turret.getAngle());
        joinedTelemetry.addData("Turret Target (deg)", robot.turret.getTargetDegrees());
        joinedTelemetry.addData("Turret On Goal", robot.turret.atTarget());
        joinedTelemetry.addData("Turret Kill Switch", robot.isTurretKilled());
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
