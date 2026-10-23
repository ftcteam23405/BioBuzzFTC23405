package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.Robot;

import org.firstinspires.ftc.teamcode.commandbase.util.Alliance;
//tune the shooter PID with Shooter.kP/kI/kD/target, re-init after changing gains
@Configurable
@TeleOp
public class ShooterPID extends OpMode {
    Robot robot;
    @Override
    public void init() {
        robot = new Robot(hardwareMap, Alliance.RED);
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {

        if (gamepad1.rightBumperWasPressed()) {
            robot.shooter.enable();
        }
        if (gamepad1.rightBumperWasReleased()) {
            robot.shooter.stop();
        }
        if (gamepad1.right_trigger > 0.5) {
            robot.intake.intakeOn();
        }
        else if (gamepad1.y)
            robot.intake.intakeReverse();

        else{
            robot.intake.intakeOff();
        }
        if (gamepad1.left_trigger_pressed) {
            robot.transfer.outtake();
        }
        else if (gamepad1.x)
            robot.transfer.intake();
        else {
            robot.transfer.stop();
        }

        robot.periodic();

        telemetry.addData("currentVelocity",robot.shooter.getVelocity());
        telemetry.addData("error",robot.shooter.getError());
        telemetry.addData("pid",robot.shooter.getOutput());
        telemetry.update();
    }

}
