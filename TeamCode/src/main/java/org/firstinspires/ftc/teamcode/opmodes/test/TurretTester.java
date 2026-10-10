package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;

// spins the turret to a position with its PID, nothing else: no pedro, no limelight, no filter
// the turret starts off so you can turn it by hand and check the encoder first
//
// dpad up/down    = target +/- bigStep       dpad left/right = target +/- smallStep (left = turret left)
// a = go to 0 (front)   x = go to leftPreset   b = go to rightPreset   y = motor off (turn it by hand)
// back = redo the encoder wrap count assuming the turret is within 36 deg of forward (motor off, turret still)
//
// setting up the encoder, with the motor off (y):
// 1. set Turret.gearRatio: turret ring teeth / encoder gear teeth (1 if the encoder is on the turret's axis)
// 2. turn the turret left by hand. ANGLE should go up, if it goes down flip Turret.encoderReversed
// 3. point the shooter straight forward, copy RAW DEG into Turret.encoderOffset, press back. ANGLE should read 0
// 4. turn the turret 90 deg left by hand (use a square), ANGLE should read 90. if it's off by a factor,
//    gearRatio is wrong. if Voltage never gets near 3.2 at the top of a turn, set maxVoltage to what it peaks at
// 5. turn it to each hard stop, put a few deg inside those numbers into Turret.minAngle / maxAngle
// then press a. if the turret runs away from the target instead of to it, flip Turret.motorReversed and restart
@Configurable
@Config
@TeleOp
public class TurretTester extends OpMode {
    public static double bigStep = 15;
    public static double smallStep = 1;
    public static double leftPreset = 90, rightPreset = -90;

    private Turret turret;
    private double target = 0;

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        turret = new Turret(hardwareMap);
    }

    @Override
    public void init_loop() {
        turret.periodic(); // off, just reads the encoder
        addTelemetry();
    }

    @Override
    public void start() {
        target = turret.getAngle(); // stay where it is until a button is pressed
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) goTo(target + bigStep);
        if (gamepad1.dpadDownWasPressed()) goTo(target - bigStep);
        if (gamepad1.dpadLeftWasPressed()) goTo(target + smallStep);
        if (gamepad1.dpadRightWasPressed()) goTo(target - smallStep);

        if (gamepad1.aWasPressed()) goTo(0);
        if (gamepad1.xWasPressed()) goTo(leftPreset);
        if (gamepad1.bWasPressed()) goTo(rightPreset);
        if (gamepad1.yWasPressed()) turret.stop();
        if (gamepad1.backWasPressed() && !turret.isEnabled()) turret.resetWraps(0);

        turret.periodic();
        addTelemetry();
    }

    private void goTo(double degrees) {
        target = Math.max(Turret.minAngle, Math.min(Turret.maxAngle, degrees));
        turret.setTargetAngle(target);
    }

    private void addTelemetry() {
        telemetry.addData("Mode", turret.isEnabled() ? "PID" : "OFF (turn by hand, a/x/b/dpad to move)");
        telemetry.addLine();
        telemetry.addData("ANGLE", "%.2f deg", turret.getAngle());
        telemetry.addData("Target", "%.2f deg", turret.isEnabled() ? turret.getTargetAngle() : target);
        telemetry.addData("Error", "%.2f deg", turret.getError());
        telemetry.addData("At target", turret.atTarget());
        telemetry.addData("Power", "%.3f", turret.getPower());
        telemetry.addData("Velocity", "%.1f deg/s", turret.getVelocity());
        telemetry.addLine();
        telemetry.addData("RAW DEG", "%.2f  <- encoderOffset", turret.getRawDegrees());
        telemetry.addData("Voltage", "%.3f / %.2f V  (%.1f turret deg per V)", turret.getVoltage(), Turret.maxVoltage, Turret.degreesPerVolt());
        if (Turret.gearRatio != 1)
            telemetry.addData("Encoder wraps", turret.getWraps());
        telemetry.addData("Limits", "%.0f to %.0f deg", Turret.minAngle, Turret.maxAngle);
        telemetry.addData("Gains", "kP %.4f  kI %.4f  kD %.4f  kS %.3f", Turret.kP, Turret.kI, Turret.kD, Turret.kS);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
