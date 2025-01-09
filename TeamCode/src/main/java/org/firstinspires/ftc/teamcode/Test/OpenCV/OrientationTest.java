package org.firstinspires.ftc.teamcode.Test.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Autonomous program for testing block orientation detection.
 */
@Autonomous(name = "Orientation Test", group = "Autonomous")
public class OrientationTest extends LinearOpMode {

    OpenCvCamera webcam;
    BlockOrientationPipeline pipeline;

    @Override
    public void runOpMode() {
        // Initialize the camera and pipeline
        initCamera();

        // Display orientation during initialization
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Block Orientation", pipeline.getOrientation());
            telemetry.update();
        }

        waitForStart();

        // Autonomous logic for orientation detection
        if (opModeIsActive()) {
            String detectedOrientation = pipeline.getOrientation();

            telemetry.addData("Detected Orientation", detectedOrientation);
            telemetry.update();

            // Placeholder for actions based on orientation
            // Example: if ("Horizontal".equals(detectedOrientation)) { /* perform action */ }
            // Example: if ("Vertical".equals(detectedOrientation)) { /* perform another action */ }

            sleep(3000); // Pause for testing
        }
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new BlockOrientationPipeline();
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

    public static class BlockOrientationPipeline extends OpenCvPipeline {

        // Visualization colors
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);

        // To store processed frame and results
        Mat processed = new Mat();
        String orientation = "Unknown";

        @Override
        public Mat processFrame(Mat input) {
            // Convert to grayscale
            Mat gray = new Mat();
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

            // Apply a binary threshold to isolate the block
            Mat binary = new Mat();
            Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Draw contours and analyze block orientation
            input.copyTo(processed);
            for (MatOfPoint contour : contours) {
                // Get bounding rectangle
                Rect boundingRect = Imgproc.boundingRect(contour);

                // Draw rectangle for visualization
                Imgproc.rectangle(processed, boundingRect, GREEN, 2);

                // Calculate aspect ratio
                double aspectRatio = (double) boundingRect.width / boundingRect.height;

                // Determine orientation
                if (aspectRatio > 1.25) {
                    orientation = "Horizontal";
                    Imgproc.putText(processed, "Horizontal", new Point(boundingRect.x, boundingRect.y - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED, 2);
                } else if (aspectRatio < 1.25) {
                    orientation = "Vertical";
                    Imgproc.putText(processed, "Vertical", new Point(boundingRect.x, boundingRect.y - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED, 2);
                } else {
                    orientation = "Unknown";
                }
            }

            return processed; // Return the processed frame
        }

        public String getOrientation() {
            return orientation;
        }
    }
}
