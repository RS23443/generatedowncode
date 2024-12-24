package org.firstinspires.ftc.teamcode.tuning.PIDTesting;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Cohen-Coon Lift Analysis with K, L, and T")
public class CohenCoonPIDAnalysis extends LinearOpMode {
    private DcMotorEx liftMotor;
    private DcMotorEx leftliftMotor;

    @Override
    public void runOpMode() {
        // Initialize motor
        liftMotor = hardwareMap.get(DcMotorEx.class, "rightslide");
        leftliftMotor = hardwareMap.get(DcMotorEx.class,"leftslide");
        leftliftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready. Press Start to begin.");
        telemetry.update();
        waitForStart();

        // Apply a constant power to the motor (step input)
        double inputPower = 0.7; // Adjust based on your lift motor setup
        liftMotor.setPower(inputPower);

        double previousPosition = liftMotor.getCurrentPosition();
        double prevleftpos = leftliftMotor.getCurrentPosition();
        double previousTime = getRuntime();

        double steadyStatePosition = 0;
        double steadyStateVelocity = 0;
        boolean steadyStateDetected = false;

        double lastVelocity = 0;
        int stabilizationCount = 0;
        final int stabilizationThreshold = 5; // Number of consecutive stable readings required
        final double velocityChangeThreshold = 2; // Threshold for change in velocity (ticks/sec)

        boolean timeDelayCaptured = false;
        double startTime = getRuntime();
        double timeDelay = 0; // L
        double timeConstant = 0; // T
        boolean timeConstantCaptured = false;
        double steadyStateTarget = 0;

        while (opModeIsActive()) {
            double currentPosition = liftMotor.getCurrentPosition();
            double lcurrpos = leftliftMotor.getCurrentPosition();
            double currentTime = getRuntime();

            // Calculate velocity (ticks per second)
            double velocity = (currentPosition - previousPosition) / (currentTime - previousTime);
            double vel_left = ((lcurrpos-prevleftpos)/(currentTime-previousTime));

            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Velocity (ticks/sec)", velocity);
            telemetry.addData("Current Position left", lcurrpos);
            telemetry.addData("Velocity Left (ticks/sec)", vel_left);
            telemetry.addData("Time Delay (L)", timeDelay);
            telemetry.addData("Time Constant (T)", timeConstant);
            telemetry.update();

            // Time Delay (L)
            if (!timeDelayCaptured && velocity > 0) {
                timeDelay = currentTime - startTime;
                timeDelayCaptured = true;
            }

            // Check if the velocity stabilizes (Steady State Detection)
            if (Math.abs(velocity - lastVelocity) < velocityChangeThreshold) {
                stabilizationCount++;
                if (stabilizationCount >= stabilizationThreshold && !steadyStateDetected) {
                    steadyStatePosition = currentPosition;
                    steadyStateVelocity = velocity;
                    steadyStateDetected = true;

                    // Process Gain (K)
                    double processGain = steadyStatePosition / inputPower;

                    telemetry.addData("Process Gain (K)", processGain);
                    telemetry.update();

                    // Target for Time Constant (63.2% of steady-state position)
                    steadyStateTarget = steadyStatePosition * 0.632;
                }
            } else {
                stabilizationCount = 0;
            }


            // Time Constant (T)
            if (steadyStateDetected && !timeConstantCaptured && currentPosition >= steadyStateTarget) {
                timeConstant = currentTime - timeDelay - startTime;
                timeConstantCaptured = true;
                telemetry.addData("Time Constant (T)", timeConstant);
                telemetry.update();
            }

            lastVelocity = velocity;
            previousPosition = currentPosition;
        }
    }
}