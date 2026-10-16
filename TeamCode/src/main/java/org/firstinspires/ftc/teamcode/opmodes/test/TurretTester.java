package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;

// spins the turret to a position (encoder ticks) with its PID, nothing else: no pedro, no limelight, no filter
// the turret starts off so you can turn it by hand and check the encoder first
//
// dpad up/down    = target +/- bigStep       dpad left/right = target +/- smallStep (left = turret left), all in ticks
// a = go to 0 (front)   x = go to leftPreset   b = go to rightPreset   y = motor off (turn it by hand)
// back = zero the encoder: wherever the turret is now becomes 0 ticks (motor off, shooter pointed straight forward)
//
// the encoder is relative (the 1150 rpm motor's own encoder), so 0 ticks is wherever the turret was when the first
// Turret was made after the app started. press back to re-zero it any time
//
// setting up the encoder, with the motor off (y):
// 1. Turret.motorTicksPerRev = 145.1 for the 1150 rpm motor, Turret.gearRatio = 5 for the 5:1 turret reduction
// 2. point the shooter straight forward, press back. POSITION should read 0
// 3. turn the turret left by hand. POSITION should go up, if it goes down change the motor direction in Turret's constructor
//    (that flips the power and the encoder together, so it can't end up fighting itself)
// 4. turn the turret 90 deg left by hand (use a square), POSITION should read ~181 ticks and ANGLE 90 deg.
//    if it's off by a factor, gearRatio or motorTicksPerRev is wrong
// 5. turn it to each hard stop, put a few ticks inside those Positions into Turret.minTicks / maxTicks
// then press a, it should drive to 0
@Configurable
@Config
@TeleOp
public class TurretTester extends OpMode {
    // ticks
    public static double bigStep = 30;   // ~15 deg
    public static double smallStep = 2;  // ~1 deg
    public static double leftPreset = 181, rightPreset = -181; // ~ +/-90 deg

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
        target = turret.getPosition(); // stay where it is until a button is pressed
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
        if (gamepad1.backWasPressed() && !turret.isEnabled()) turret.resetPosition();

        turret.periodic();
        addTelemetry();
    }

    private void goTo(double ticks) {
        target = Math.max(Turret.minTicks, Math.min(Turret.maxTicks, ticks));
        turret.setTarget(target);
    }

    private void addTelemetry() {
        telemetry.addData("Mode", turret.isEnabled() ? "PID" : "OFF (turn by hand, a/x/b/dpad to move)");
        telemetry.addLine();
        telemetry.addData("POSITION", "%.1f ticks  (%.2f deg)", turret.getPosition(), turret.getAngle());
        telemetry.addData("Target", "%.1f ticks", turret.isEnabled() ? turret.getTarget() : target);
        telemetry.addData("Error", "%.1f ticks", turret.getError());
        telemetry.addData("At target", turret.atTarget());
        telemetry.addData("Power", "%.3f", turret.getPower());
        telemetry.addData("Velocity", "%.1f ticks/s", turret.getVelocity());
        telemetry.addLine();
        telemetry.addData("RAW TICKS", "%d  (zero at %.0f)", turret.getRawTicks(), turret.getZeroTicks());
        telemetry.addData("Ticks per deg", "%.3f  (%.1f per turret turn)", Turret.degreesToTicks(1), Turret.degreesToTicks(360));
        telemetry.addData("Limits", "%.0f to %.0f ticks", Turret.minTicks, Turret.maxTicks);
        telemetry.addData("Gains", "kP %.4f  kI %.4f  kD %.4f  kS %.3f", Turret.kP, Turret.kI, Turret.kD, Turret.kS);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
