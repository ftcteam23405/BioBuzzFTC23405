package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;


// dpad up/down = target +/- bigStep   dpad left/right = target +/- smallStep (left = turret left)
// a = 0 (forward)   x = leftPreset   b = rightPreset   y = motor off
// back = zero here (motor off)   start = resync from the abs encoder (motor off)
//
// ANGLE / POSITION come from the motor encoder (what the PID uses). ABS ANGLE comes from the analog encoder,
// which is only used to set the starting position. see the setup steps in the tuning guide
@Configurable
@Config
@TeleOp
public class TurretTester extends OpMode {
    public static double bigStep = 15;   // deg
    public static double smallStep = 1;  // deg
    public static double leftPreset = 90, rightPreset = -90;

    private Turret turret;
    private double target = 0; // deg
    private double minVoltSeen = Double.NaN, maxVoltSeen = Double.NaN;

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        turret = new Turret(hardwareMap);
    }

    @Override
    public void init_loop() {
        if (gamepad1.backWasPressed()) turret.zeroHere();
        if (gamepad1.startWasPressed()) turret.resync();
        update();
    }

    @Override
    public void start() {
        target = turret.getAngle();
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
        if (!turret.isEnabled()) {
            if (gamepad1.backWasPressed()) turret.zeroHere();
            if (gamepad1.startWasPressed()) turret.resync();
        }

        update();
    }

    private void goTo(double degrees) {
        target = Math.max(Turret.ticksToDegrees(Turret.minTicks), Math.min(Turret.ticksToDegrees(Turret.maxTicks), degrees));
        turret.setTargetDegrees(target);
    }

    private void update() {
        turret.periodic();
        turret.readAbsolute();

        double v = turret.getVoltage();
        if (Double.isNaN(minVoltSeen) || v < minVoltSeen) minVoltSeen = v;
        if (Double.isNaN(maxVoltSeen) || v > maxVoltSeen) maxVoltSeen = v;

        telemetry.addData("Mode", turret.isEnabled() ? "PID" : "OFF (turn by hand)");
        telemetry.addLine();
        telemetry.addLine("--- motor encoder (control) ---");
        telemetry.addData("ANGLE", "%.2f deg", turret.getAngle());
        telemetry.addData("POSITION", "%.0f ticks  (raw motor %.0f)", turret.getPosition(), turret.getMotorTicks());
        telemetry.addData("Target", "%.2f deg  (%.0f ticks)", turret.isEnabled() ? turret.getTargetDegrees() : target,
                turret.isEnabled() ? turret.getTarget() : Turret.degreesToTicks(target));
        telemetry.addData("Error", "%.1f ticks  (%.2f deg)", turret.getError(), Turret.ticksToDegrees(turret.getError()));
        telemetry.addData("At target", turret.atTarget());
        telemetry.addData("Power", "%.3f", turret.getPower());
        telemetry.addData("Velocity", "%.0f ticks/s", turret.getVelocity());
        telemetry.addLine();
        telemetry.addLine("--- abs encoder (startup only) ---");
        telemetry.addData("ABS ANGLE", "%.2f deg  (only right within +/-%.0f deg of forward)",
                turret.getAbsoluteAngle(), 180 / Turret.encoderGearRatio);
        telemetry.addData("ABS CHECK", "%.2f deg  (should stay ~0 everywhere)", turret.getAbsoluteCheck());
        telemetry.addData("ENCODER", "%.1f deg  (offset %.1f)", turret.getEncoderDegrees(), Turret.encoderOffset);
        telemetry.addData("VOLTAGE", "%.3f V  (seen %.3f to %.3f, set %.2f to %.2f)", v, minVoltSeen, maxVoltSeen,
                Turret.minVoltage, Turret.maxVoltage);
        telemetry.addLine();
        telemetry.addData("Ratios", "%.1f motor ticks/rev x %.2f = %.1f ticks/turret turn, abs %.2f:1",
                Turret.motorTicksPerRev, Turret.gearRatio, Turret.ticksPerTurretRev(), Turret.encoderGearRatio);
        telemetry.addData("Reversed", "motor %b  abs %b", Turret.motorReversed, Turret.encoderReversed);
        telemetry.addData("Limits", "%.0f to %.0f ticks", Turret.minTicks, Turret.maxTicks);
        telemetry.addData("Gains", "kP %.4f  kD %.5f  kS %.3f", Turret.kP, Turret.kD, Turret.kS);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
