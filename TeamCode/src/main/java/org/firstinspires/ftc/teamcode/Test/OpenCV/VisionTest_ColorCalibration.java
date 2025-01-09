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

        static final int CROP_WIDTH = 160; // Width of the cropped area
        static final int CROP_HEIGHT = 120; // Height of the cropped area
        static final int REGION_WIDTH = 40; // Width of the region to analyze
        static final int REGION_HEIGHT = 40; // Height of the region to analyze

        Point region_pointA, region_pointB;
        Mat HSV = new Mat();
        Mat croppedInput, region_HSV;
        Scalar averageHSV = new Scalar(0, 0, 0);

        @Override
        public Mat processFrame(Mat input) {
            // Define the cropped region (center 160x120 area)
            int cropWidth = 160;
            int cropHeight = 120;

            int offsetX = 20; // Shift right (+) or left (-)
            int offsetY = -10; // Shift down (+) or up (-)

            int startX = ((input.width() - cropWidth) / 2) + offsetX;
            int startY = ((input.height() - cropHeight) / 2) + offsetY;

            // Ensure cropping doesn't exceed frame bounds
            startX = Math.max(0, Math.min(startX, input.width() - cropWidth));
            startY = Math.max(0, Math.min(startY, input.height() - cropHeight));
            Rect cropRect = new Rect(startX, startY, CROP_WIDTH, CROP_HEIGHT);

            // Crop the input image
            croppedInput = input.submat(cropRect);

            // Define the region within the cropped image
            int regionStartX = (CROP_WIDTH - REGION_WIDTH) / 2;
            int regionStartY = (CROP_HEIGHT - REGION_HEIGHT) / 2;

            region_pointA = new Point(regionStartX, regionStartY);
            region_pointB = new Point(regionStartX + REGION_WIDTH, regionStartY + REGION_HEIGHT);

            // Convert the cropped input to HSV
            Imgproc.cvtColor(croppedInput, HSV, Imgproc.COLOR_RGB2HSV);

            // Extract the region of interest within the cropped input
            Rect regionRect = new Rect(region_pointA, region_pointB);
            region_HSV = HSV.submat(regionRect);

            // Compute the average HSV in the region
            averageHSV = Core.mean(region_HSV);

            // Draw a rectangle around the region for visualization
            Imgproc.rectangle(croppedInput, region_pointA, region_pointB, new Scalar(0, 255, 0), 2);

            // Return the cropped frame for display
            return croppedInput;
        }

        public Scalar getAverageHSV() {
            return averageHSV;
        }
    }
}
