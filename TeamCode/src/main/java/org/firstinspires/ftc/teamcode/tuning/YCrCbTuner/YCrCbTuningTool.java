package org.firstinspires.ftc.teamcode.tuning.YCrCbTuner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "YCrCb Tuning Tool", group = "Testing")
public class YCrCbTuningTool extends LinearOpMode {
    private OpenCvCamera webcam;
    private Scalar lowerYCrCb = new Scalar(0, 100, 100); // Default lower bound
    private Scalar upperYCrCb = new Scalar(255, 200, 200); // Default upper bound

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set up the pipeline
        webcam.setPipeline(new TuningPipeline());

        // Open the webcam and start streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Error code: " + errorCode);
                telemetry.update();
            }
        });

        // Wait for the start command
        telemetry.addLine("Press Play to start tuning YCrCb values...");
        telemetry.update();
        waitForStart();

        // Loop for tuning
        while (opModeIsActive()) {
            // Adjust bounds using gamepad buttons
            if (gamepad1.dpad_up) lowerYCrCb.val[1] += 1; // Increase Cr lower bound
            if (gamepad1.dpad_down) lowerYCrCb.val[1] -= 1; // Decrease Cr lower bound
            if (gamepad1.dpad_left) lowerYCrCb.val[2] -= 1; // Decrease Cb lower bound
            if (gamepad1.dpad_right) lowerYCrCb.val[2] += 1; // Increase Cb lower bound

            if (gamepad1.a) upperYCrCb.val[1] += 1; // Increase Cr upper bound
            if (gamepad1.b) upperYCrCb.val[1] -= 1; // Decrease Cr upper bound
            if (gamepad1.x) upperYCrCb.val[2] -= 1; // Decrease Cb upper bound
            if (gamepad1.y) upperYCrCb.val[2] += 1; // Increase Cb upper bound

            // Clamp values to avoid invalid ranges
            lowerYCrCb.val[1] = Math.max(0, Math.min(255, lowerYCrCb.val[1]));
            lowerYCrCb.val[2] = Math.max(0, Math.min(255, lowerYCrCb.val[2]));
            upperYCrCb.val[1] = Math.max(0, Math.min(255, upperYCrCb.val[1]));
            upperYCrCb.val[2] = Math.max(0, Math.min(255, upperYCrCb.val[2]));

            // Display current YCrCb ranges on telemetry
            telemetry.addData("Lower YCrCb", "Y: %.0f, Cr: %.0f, Cb: %.0f",
                    lowerYCrCb.val[0], lowerYCrCb.val[1], lowerYCrCb.val[2]);
            telemetry.addData("Upper YCrCb", "Y: %.0f, Cr: %.0f, Cb: %.0f",
                    upperYCrCb.val[0], upperYCrCb.val[1], upperYCrCb.val[2]);
            telemetry.update();

            sleep(50); // Short delay to avoid rapid changes
        }

        // Stop streaming when done
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    private class TuningPipeline extends OpenCvPipeline {
        private Mat yCrCb = new Mat(); // Persistent Mat for YCrCb conversion
        private Mat mask = new Mat();  // Persistent Mat for mask
        private Mat output = new Mat(); // Persistent Mat for output

        @Override
        public Mat processFrame(Mat input) {
            // Convert the input frame to YCrCb
            Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_BGR2YCrCb);

            // Create a mask based on current YCrCb thresholds
            Core.inRange(yCrCb, lowerYCrCb, upperYCrCb, mask);

            // Apply the mask to the original frame
            Core.bitwise_and(input, input, output, mask);

            // Return the processed frame
            return output;
        }

        @Override
        public void finalize() {
            // Release Mats when the pipeline is destroyed
            yCrCb.release();
            mask.release();
            output.release();
        }
    }
}
