package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
@TeleOp
public class DriveDirection extends OpMode {

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    public DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    @Override
    public void loop() {

        telemetryM.debug("Select Motor");
        telemetryM.debug("Run frontLeftMotor select A");
        telemetryM.debug("Run backLeftMotor select B");
        telemetryM.debug("Run frontRightMotor select X");
        telemetryM.debug("Run backRightMotor select Y");

        if (gamepad1.a){
            frontLeftMotor.setPower(1);
            telemetryM.debug("frontLeftMotor running: ", frontLeftMotor.getPower());
        }
        if (gamepad1.b){
            backLeftMotor.setPower(1);
            telemetryM.debug("backLeftMotor running: ", backLeftMotor.getPower());
        }
        if (gamepad1.x){
            frontRightMotor.setPower(1);
            telemetryM.debug("frontRightMotor running: ", frontRightMotor.getPower());
        }
        if (gamepad1.y){
            backRightMotor.setPower(1);
            telemetryM.debug("backRightMotor running: ", backRightMotor.getPower());
        }
        if (gamepad1.right_bumper){ //stops all motors
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }

        telemetryM.update(telemetry);
    }
}

