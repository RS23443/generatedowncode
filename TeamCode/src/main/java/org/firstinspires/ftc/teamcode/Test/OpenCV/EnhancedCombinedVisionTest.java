package org.firstinspires.ftc.teamcode.Test.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Enhanced Vision Test with Centered Cropping", group = "Autonomous")
public class EnhancedCombinedVisionTest extends LinearOpMode {

    OpenCvCamera webcam;
    EnhancedCombinedDetectionPipeline pipeline;

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

        // Test logic during autonomous mode
        if (opModeIsActive()) {
            for (int i = 0; i < 5; i++) { // Example: Loop to process 5 blocks
                String detectedColor = pipeline.getDominantColor();
                String detectedOrientation = pipeline.getOrientation();

                telemetry.addData("Detected Color", detectedColor);
                telemetry.addData("Detected Orientation", detectedOrientation);
                telemetry.update();

                if ("Red".equals(detectedColor) && "Horizontal".equals(detectedOrientation)) {
                    telemetry.addLine("Action: Performing Red Horizontal Action");
                } else if ("Yellow".equals(detectedColor) && "Vertical".equals(detectedOrientation)) {
                    telemetry.addLine("Action: Performing Yellow Vertical Action");
                } else if ("Blue".equals(detectedColor)) {
                    telemetry.addLine("Action: Blue block detected");
                } else {
                    telemetry.addLine("Action: No valid block detected");
                }
                telemetry.update();

                sleep(2000); // Pause to simulate time for the next block to appear
            }
        }
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new EnhancedCombinedDetectionPipeline();
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

    /**
     * Pipeline with centered cropping and aspect ratio-based orientation detection
     */
    public static class EnhancedCombinedDetectionPipeline extends OpenCvPipeline {

        // Visualization colors
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);

        // Block attributes
        String dominantColor = "Unknown";
        String orientation = "Unknown";

        @Override
        public Mat processFrame(Mat input) {
            // Define the cropping region (focus on the middle of the field)
            int cropWidth = 80;  // Width of the cropped region
            int cropHeight = 60; // Height of the cropped region
            int cropStartX = (input.width() - cropWidth) / 2;  // Centered horizontally
            int cropStartY = (input.height() - cropHeight) / 2; // Centered vertically

            Rect cropRect = new Rect(cropStartX, cropStartY, cropWidth, cropHeight);
            Mat croppedField = input.submat(cropRect);

            // Convert cropped field to grayscale for contour detection
            Mat gray = new Mat();
            Imgproc.cvtColor(croppedField, gray, Imgproc.COLOR_RGB2GRAY);

            // Threshold the grayscale image
            Mat binary = new Mat();
            Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY);

            // Find contours in the binary image
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                MatOfPoint largestContour = contours.get(0);
                double largestArea = 0;

                // Find the largest contour
                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area > largestArea) {
                        largestArea = area;
                        largestContour = contour;
                    }
                }

                // Calculate the bounding rectangle
                Rect boundingRect = Imgproc.boundingRect(largestContour);
                double aspectRatio = (double) boundingRect.width / boundingRect.height;

                // Determine orientation based on aspect ratio
                if (aspectRatio > 1.3) {
                    orientation = "Horizontal";
                } else if (aspectRatio < 1.3) {
                    orientation = "Vertical";
                } else {
                    orientation = "Diagonal";
                }

                // Determine color
                Mat hsv = new Mat();
                Imgproc.cvtColor(croppedField, hsv, Imgproc.COLOR_RGB2HSV);
                Mat blockRegion = hsv.submat(boundingRect);
                Scalar avgHSV = Core.mean(blockRegion);
                dominantColor = detectColor(avgHSV);
                blockRegion.release();

                // Draw the bounding rectangle and annotate it
                Point rectStart = new Point(cropStartX + boundingRect.x, cropStartY + boundingRect.y);
                Point rectEnd = new Point(rectStart.x + boundingRect.width, rectStart.y + boundingRect.height);
                Imgproc.rectangle(input, rectStart, rectEnd, GREEN, 2);

                Imgproc.putText(input, "Color: " + dominantColor,
                        new Point(rectStart.x, rectStart.y - 20),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED, 2);
                Imgproc.putText(input, "Orientation: " + orientation,
                        new Point(rectStart.x, rectStart.y - 5),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED, 2);
            }

            return input;
        }

        /**
         * Classify color based on average HSV values.
         */
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
