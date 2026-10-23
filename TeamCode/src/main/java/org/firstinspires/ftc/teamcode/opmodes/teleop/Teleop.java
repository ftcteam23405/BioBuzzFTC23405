package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
import org.firstinspires.ftc.teamcode.commandbase.Robot;

@Config
@Configurable

public class Teleop extends OpMode {

    JoinedTelemetry joinedTelemetry;
    Controller controller;

    Robot robot;
    final Alliance alliance;
    public boolean hold = false;
    public static double kP = 0.0345;
    public static double kI = 0;
    public static double kD = 0.02;
    double previousVelocity = 0;
    double previousTime = 0;
    public static double target = 160;
    private VoltageSensor voltageSensor;

    public Teleop(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() { //what happens at initialization
        robot = new Robot(hardwareMap, alliance);
        robot.setPose(robot.getEnd()); //also resets the shooter's distance filter
        controller = Controller.pid(kP,kI,kD);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
    }

    @Override
    public void init_loop() { //what happens when looping on init

        joinedTelemetry.addData("Robot Saved Pose", robot.getEnd()); //way to check saved pose after auto
        joinedTelemetry.update();
    }

    @Override
    public void start() { //what happens when start is pressed

    }

    @Override
    public void loop() { //what happens during the whole match - what runs in the background
        Scheduler.execute();
        double currentVelocity = robot.shooter.getVelocity();
        double currentTime = getRuntime();

        double derivative = ((currentVelocity-previousVelocity)/(currentTime-previousTime))/1000.0;
        double output = controller.calculate(target,target-currentVelocity,derivative);
        output = output * (12/voltageSensor.getVoltage());


        DrivePowers powers = ManualDrive.fieldCentric(
            -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, robot.follower.pose().heading(), (robot.alliance == Alliance.BLUE ? Math.toRadians(180) : Math.toRadians(0))
        );

        robot.follower.manual(powers);
        if (gamepad1.startWasPressed())
            robot.resetHeading();

        if (gamepad1.rightBumperWasPressed()) {
            robot.shooter.setPower(limit(output));
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
            robot.transfer.outtake();
        if (gamepad1.bWasPressed())
            robot.transfer.intake();


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
    private double limit(double n) {
        if (n>1)
            return 1;
        else if (n<-1)
            return -1;
        else
            return n;
    }
    public void updateTelemetry() {
        joinedTelemetry.addData("LoopTime Hz", robot.getLoopTimeHz());
        joinedTelemetry.addLine();
        joinedTelemetry.addData("Follower Pose", robot.follower.pose().toString());
        joinedTelemetry.addLine();
        joinedTelemetry.addData("Intake", robot.intake.getCurrent());
        joinedTelemetry.addData("Shooter", robot.shooter.getCurrent());
        joinedTelemetry.addData("Drivetrain", robot.driveTrain.getCurrent());
        joinedTelemetry.addData("FL", robot.driveTrain.fl.getCurrent(CurrentUnit.MILLIAMPS));
        joinedTelemetry.addData("FR", robot.driveTrain.fr.getCurrent(CurrentUnit.MILLIAMPS));
        joinedTelemetry.addData("BL", robot.driveTrain.bl.getCurrent(CurrentUnit.MILLIAMPS));
        joinedTelemetry.addData("BR", robot.driveTrain.br.getCurrent(CurrentUnit.MILLIAMPS));

        for (LynxModule hub : robot.hubs) {
            joinedTelemetry.addData(hub.getDeviceName(), hub.getCurrent(CurrentUnit.MILLIAMPS)+", "+hub.getInputVoltage(VoltageUnit.MILLIVOLTS));
        }
        joinedTelemetry.addData("Hold Position", hold);
        joinedTelemetry.update();
    }
}
