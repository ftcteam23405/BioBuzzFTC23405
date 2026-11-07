package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;


// hold right bumper = spin the shooter at velocity, let go = stop
// dpad up/down = velocity +/- step
// y = intake on   b = intake reverse   a = intake off (speeds are in Intake in panels)
// change velocity in panels while holding and it updates live. graph Target and Velocity in panels to watch it
@Configurable
@Config
@TeleOp
public class ShooterTester extends OpMode {
    public static double velocity = 1300; // ticks/s
    public static double step = 50;       // ticks/s per dpad press

    private JoinedTelemetry joinedTelemetry;
    private Shooter shooter;
    private Intake intake;

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) velocity += step;
        if (gamepad1.dpadDownWasPressed()) velocity = Math.max(0, velocity - step);

        // set every loop so changing velocity in panels takes effect while holding
        if (gamepad1.right_bumper)
            shooter.setTargetVelocity(velocity);
        else
            shooter.stop();

        if (gamepad1.yWasPressed()) intake.intakeOn();
        if (gamepad1.bWasPressed()) intake.intakeReverse();
        if (gamepad1.aWasPressed()) intake.intakeOff();

        shooter.periodic();
        addTelemetry();
    }

    private void addTelemetry() {
        joinedTelemetry.addData("Set Velocity", velocity);
        joinedTelemetry.addData("Target", shooter.getTargetVelocity());
        joinedTelemetry.addData("Velocity", shooter.getVelocity());
        joinedTelemetry.addData("Error", shooter.getTargetVelocity() - shooter.getVelocity());
        joinedTelemetry.addData("At Target", shooter.atTarget());
        joinedTelemetry.addData("Power", shooter.getPower());
        joinedTelemetry.addData("Battery", String.format("%.2f V", shooter.getBatteryVoltage()));
        joinedTelemetry.addLine();
        joinedTelemetry.addData("Intake Full", intake.isFull());
        joinedTelemetry.addLine(intake.getCurrent());
        joinedTelemetry.addLine();
        joinedTelemetry.addData("Gains", String.format("kS %.4f  kV %.7f  kP %.6f", Shooter.kS, Shooter.kV, Shooter.kP));
        joinedTelemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.intakeOff();
    }
}
