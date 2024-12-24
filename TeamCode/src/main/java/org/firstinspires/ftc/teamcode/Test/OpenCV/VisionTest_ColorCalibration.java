package org.firstinspires.ftc.teamcode.Test.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Autonomous program to test block color detection using OpenCV.
 */

// works - these are the values for yellow, red, and blue
@Autonomous(name = "Vision Test Color Calibration", group = "Autonomous")
public class VisionTest_ColorCalibration extends LinearOpMode {

    OpenCvCamera webcam;
    ColorDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        // Initialize the camera
        initCamera();

        // Wait for start while showing live detection updates
        while (!isStarted() && !isStopRequested()) {
            Scalar avgHSV = pipeline.getAverageHSV();
            telemetry.addData("Average HSV", avgHSV);
            telemetry.addData("Detected Color", getDominantColor(avgHSV));
            telemetry.update();
        }

        waitForStart();

        // Autonomous logic (test detection here)
        if (opModeIsActive()) {
            Scalar detectedHSV = pipeline.getAverageHSV();
            String detectedColor = getDominantColor(detectedHSV);

            // Display detected color after start
            telemetry.addData("Detected Color", detectedColor);
            telemetry.update();

            // Insert specific autonomous actions based on color detection
            // Example: if ("Red".equals(detectedColor)) { /* perform action */ }
            // Example: if ("Blue".equals(detectedColor)) { /* perform another action */ }

            sleep(3000); // Pause to allow for visual confirmation in telemetry
        }
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new ColorDetectionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });
    }

    private String getDominantColor(Scalar avgHSV) {
        double hue = avgHSV.val[0];
        if (hue < 15 || hue > 160) {
            return "Red";
        } else if (hue > 15 && hue < 40) {
            return "Yellow";
        } else if (hue > 100 && hue < 140) {
            return "Blue";
        }
        return "Unknown";
    }

    public static class ColorDetectionPipeline extends OpenCvPipeline {

        static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(140, 100); // Center position
        static final int REGION_WIDTH = 40;
        static final int REGION_HEIGHT = 40;

        Point region_pointA = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x,
                REGION_TOPLEFT_ANCHOR_POINT.y);
        Point region_pointB = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region_HSV, HSV = new Mat();
        Scalar averageHSV = new Scalar(0, 0, 0);

        @Override
        public void init(Mat firstFrame) {
            Imgproc.cvtColor(firstFrame, HSV, Imgproc.COLOR_RGB2HSV);
            region_HSV = HSV.submat(new Rect(region_pointA, region_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            averageHSV = Core.mean(region_HSV);

            // Draw rectangle for visualization
            Imgproc.rectangle(input, region_pointA, region_pointB, new Scalar(0, 255, 0), 2);
            return input;
        }

        public Scalar getAverageHSV() {
            return averageHSV;
        }
    }
}
