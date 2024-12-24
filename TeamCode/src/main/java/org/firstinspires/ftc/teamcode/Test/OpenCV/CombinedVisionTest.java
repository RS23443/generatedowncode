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
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Autonomous program for testing combined block color and orientation detection.
 */
@Autonomous(name = "Combined Vision Test", group = "Autonomous")
public class CombinedVisionTest extends LinearOpMode {

    OpenCvCamera webcam;
    CombinedDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        // Initialize the camera and pipeline
        initCamera();

        // Display color and orientation during initialization
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Block Color", pipeline.getDominantColor());
            telemetry.addData("Block Orientation", pipeline.getOrientation());
            telemetry.update();
        }

        waitForStart();

        // Autonomous logic for testing combined detection
        if (opModeIsActive()) {
            String detectedColor = pipeline.getDominantColor();
            String detectedOrientation = pipeline.getOrientation();

            telemetry.addData("Detected Color", detectedColor);
            telemetry.addData("Detected Orientation", detectedOrientation);
            telemetry.update();

            // Placeholder for actions based on combined detection
            // Example: if ("Red".equals(detectedColor) && "Vertical".equals(detectedOrientation)) { /* perform action */ }
            // Example: if ("Blue".equals(detectedColor) && "Horizontal".equals(detectedOrientation)) { /* perform another action */ }

            sleep(3000); // Pause for testing
        }
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CombinedDetectionPipeline();
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

    public static class CombinedDetectionPipeline extends OpenCvPipeline {

        // Visualization colors
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);

        // Region for color detection
        static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(140, 100);
        static final int REGION_WIDTH = 40;
        static final int REGION_HEIGHT = 40;

        Point region_pointA = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x,
                REGION_TOPLEFT_ANCHOR_POINT.y);
        Point region_pointB = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat HSV = new Mat();
        Mat region_HSV;
        Scalar averageHSV = new Scalar(0, 0, 0);

        String dominantColor = "Unknown";
        String orientation = "Unknown";

        @Override
        public void init(Mat firstFrame) {
            // Convert first frame to HSV for color analysis
            Imgproc.cvtColor(firstFrame, HSV, Imgproc.COLOR_RGB2HSV);
            region_HSV = HSV.submat(new Rect(region_pointA, region_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            // Detect color
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            averageHSV = Core.mean(region_HSV);
            dominantColor = detectColor(averageHSV);

            // Detect orientation
            Mat gray = new Mat();
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

            Mat binary = new Mat();
            Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                // Get bounding rectangle
                Rect boundingRect = Imgproc.boundingRect(contour);

                // Draw rectangle for visualization
                Imgproc.rectangle(input, boundingRect, GREEN, 2);

                // Calculate aspect ratio for orientation
                double aspectRatio = (double) boundingRect.width / boundingRect.height;
                if (aspectRatio > 1.25) {
                    orientation = "Horizontal";
                    Imgproc.putText(input, "Horizontal", new Point(boundingRect.x, boundingRect.y - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED, 2);
                } else if (aspectRatio < 1.25) {
                    orientation = "Vertical";
                    Imgproc.putText(input, "Vertical", new Point(boundingRect.x, boundingRect.y - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED, 2);
                } else {
                    orientation = "Unknown";
                }
            }

            return input; // Return the frame with visual annotations
        }

        private String detectColor(Scalar avgHSV) {
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

        public String getDominantColor() {
            return dominantColor;
        }

        public String getOrientation() {
            return orientation;
        }
    }
}

