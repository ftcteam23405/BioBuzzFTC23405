package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Turret;


// dpad up/down = target +/- bigStep   dpad left/right = target +/- smallStep (left = turret left)
// a = 0 (forward)   x = leftPreset   b = rightPreset   y = motor off
// back = zero here (motor off)   start = resync (motor off)
//
// SETUP (motor off, use y):
// 1. Plug the encoder's 3-pin analog output into an analog port, name it "encoder" in the config.
//    Set Turret.gearRatio to encoder turns per turret turn (5 if it's on the 5:1 input, 1 if on the turret).
// 2. Spin the turret slowly by hand. RAW should sweep 0-4096 and VOLTAGE 0-3.2 V. If the voltage tops out
//    lower, set Turret.maxVoltage to that.
// 3. Turn the turret left. ANGLE should go up. If not, flip Turret.encoderReversed.
// 4. Point the shooter straight forward, press back. Copy RAW into Turret.encoderOffset in Turret.java.
// 5. Turn it 90 deg left by hand, ANGLE should read ~90 (5120 ticks). If it's off by a factor, gearRatio is wrong.
// 6. Press a. If it drives away from 0, flip the motor direction in Turret's constructor.
// 7. Turn to each hard stop and put a few deg inside those POSITIONs (ticks) into Turret.minTicks / maxTicks.
// At 5:1 the encoder repeats every 72 turret deg, so the turret must be within 36 deg of forward when the
// robot powers on. If ANGLE is off by a multiple of 72, point it forward and press start.
@Configurable
@Config
@TeleOp
public class TurretTester extends OpMode {
    public static double bigStep = 15;   // deg
    public static double smallStep = 1;  // deg
    public static double leftPreset = 90, rightPreset = -90;

    private Turret turret;
    private double target = 0; // deg

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
        turret.periodic();
        addTelemetry();
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

        turret.periodic();
        addTelemetry();
    }

    private void goTo(double degrees) {
        target = Math.max(Turret.ticksToDegrees(Turret.minTicks), Math.min(Turret.ticksToDegrees(Turret.maxTicks), degrees));
        turret.setTargetDegrees(target);
    }

    private void addTelemetry() {
        telemetry.addData("Mode", turret.isEnabled() ? "PID" : "OFF (turn by hand)");
        telemetry.addLine();
        telemetry.addData("ANGLE", "%.2f deg", turret.getAngle());
        telemetry.addData("POSITION", "%.0f ticks", turret.getPosition());
        telemetry.addData("Target", "%.2f deg  (%.0f ticks)", turret.isEnabled() ? turret.getTargetDegrees() : target,
                turret.isEnabled() ? turret.getTarget() : Turret.degreesToTicks(target));
        telemetry.addData("Error", "%.0f ticks  (%.2f deg)", turret.getError(), Turret.ticksToDegrees(turret.getError()));
        telemetry.addData("At target", turret.atTarget());
        telemetry.addData("Power", "%.3f", turret.getPower());
        telemetry.addData("Velocity", "%.0f ticks/s", turret.getVelocity());
        telemetry.addLine();
        telemetry.addData("RAW", "%.0f ticks  (offset %.0f)", turret.getRawTicks(), Turret.encoderOffset);
        telemetry.addData("VOLTAGE", "%.3f V  (max %.2f)", turret.getVoltage(), Turret.maxVoltage);
        telemetry.addData("Gear ratio", "%.2f  reversed %b  %.1f ticks/deg", Turret.gearRatio, Turret.encoderReversed, Turret.degreesToTicks(1));
        telemetry.addData("Limits", "%.0f to %.0f ticks", Turret.minTicks, Turret.maxTicks);
        telemetry.addData("Gains", "kP %.6f  kD %.7f  kS %.3f", Turret.kP, Turret.kD, Turret.kS);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
