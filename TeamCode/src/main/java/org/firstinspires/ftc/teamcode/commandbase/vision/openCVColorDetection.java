package org.firstinspires.ftc.teamcode.commandbase.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@TeleOp(name = "OpenCV Color Detection")

public class openCVColorDetection extends LinearOpMode {

    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;

    public static final double objectWidthInRealWorldUnits = 3.75;
    public static final double focalLength = 728;

    @Override
    public void runOpMode() {

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.update();
        }

        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new ColorDetectionPipeline());
        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    class ColorDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_BGR2HSV);

            // Masks for red, yellow, and blue
            Mat redMask = getColorMask(hsvFrame, new Scalar(0, 100, 100), new Scalar(10, 255, 255));
            Mat yellowMask = getColorMask(hsvFrame, new Scalar(25, 100, 100), new Scalar(35, 255, 255));
            Mat blueMask = getColorMask(hsvFrame, new Scalar(100, 100, 100), new Scalar(140, 255, 255));

            // Count non-zero pixels for each mask
            int redCount = Core.countNonZero(redMask);
            int yellowCount = Core.countNonZero(yellowMask);
            int blueCount = Core.countNonZero(blueMask);

            // Determine the dominant color
            String detectedColor = "None";
            if (redCount > yellowCount && redCount > blueCount) {
                detectedColor = "Red";
            } else if (yellowCount > redCount && yellowCount > blueCount) {
                detectedColor = "Yellow";
            } else if (blueCount > redCount && blueCount > yellowCount) {
                detectedColor = "Blue";
            }

            // Send telemetry for the detected color
            telemetry.addData("Detected Color", detectedColor);

            return input;
        }

        private Mat getColorMask(Mat frame, Scalar lowerBound, Scalar upperBound) {
            Mat mask = new Mat();
            Core.inRange(frame, lowerBound, upperBound, mask);
            return mask;
        }
    }

    private static double getDistance(double width) {
        return (objectWidthInRealWorldUnits * focalLength) / width;
    }
}
