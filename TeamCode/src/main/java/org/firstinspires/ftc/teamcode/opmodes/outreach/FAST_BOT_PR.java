package org.firstinspires.ftc.teamcode.opmodes.outreach;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.tuning.Globals.*;
/**
 * TeleOp mode for controlling the robot. Integrates driving, arm, slide, and intake systems.
 * Uses PIDF controllers to manage motor movements for precise positioning.
 */
@Config
@Configurable
@TeleOp
@Disabled
public class FAST_BOT_PR extends LinearOpMode {

    private DcMotor rightMotor, leftMotor;
    private Servo gateServo;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        initializeHardware();

        // Reset IMU orientation
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(imuParameters);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
        // Wait for start
        telemetry.addLine("Initialized. Ready to start.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            handleDriving();
            gateControl();
            telemetry.update();
        }
    }

    /**
     * Initializes the hardware components and PIDF controllers.
     */
    private void initializeHardware() {
        // Initialize motors and servos
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");

        gateServo = hardwareMap.get(Servo.class, "gateServo");

        // Set directions for drivetrain motors
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
    }

    private void handleDriving() {

        // Set motor powers
        double y = (gamepad1.left_stick_y); // Remember, Y stick is reversed!
        double rx = (gamepad1.left_stick_x);

        leftMotor.setPower(y - rx);
        rightMotor.setPower(y + rx);
    }

    private void gateControl() {

        if (gamepad1.a) {
            gateServo.setPosition(GATE_CLOSE_POS);
        } else if (gamepad1.y) {
            gateServo.setPosition(GATE_OPEN_POS);
        }

    }
}