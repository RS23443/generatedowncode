package org.firstinspires.ftc.teamcode.tuning.PIDTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Two Slide Motion Profiling")
@Config
@Disabled
public class motionprofilingattempt1 extends LinearOpMode {

    private DcMotorEx rightSlide;
    private DcMotorEx leftSlide;

    // PID coefficients (tune these values)
    private static final PIDFCoefficients PID_COEFFICIENTS = new PIDFCoefficients(0.017, 0.0025, 0.0001, 0.01);

    // Motion profiling parameters
    private static final double MAX_VELOCITY_TICKS = 2796; // ticks/sec
    private static final double MAX_ACCELERATION_TICKS = 500; // ticks/sec^2 (arbitrary, tune for your setup)
    private static final double TARGET_POSITION_TICKS = 2000; // Example: Move 10 revolutions

    private static final double MAX_POSITION = ((4498+4345)/2);
    @Override
    public void runOpMode() {
        // Initialize hardware
        initSlides(hardwareMap);

        telemetry.addLine("Ready. Press Start.");
        telemetry.update();
        waitForStart();

        // Test motion profiling
        motionProfileMove(TARGET_POSITION_TICKS);

        // Stop the slides
        rightSlide.setPower(0);
        leftSlide.setPower(0);

        telemetry.addLine("Motion Complete.");
        telemetry.update();
        sleep(2000);
    }

    private void initSlides(HardwareMap hardwareMap) {
        // Initialize motors
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightslide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftslide");
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);


        // Reset encoders
        //rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run without encoders initially
        rightSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set PID coefficients
        rightSlide.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PID_COEFFICIENTS);
        leftSlide.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PID_COEFFICIENTS);

        telemetry.addLine("Slides Initialized.");
        telemetry.update();
    }

    private void motionProfileMove(double targetPosition) {
        double currentRightPosition = rightSlide.getCurrentPosition();
        double currentLeftPosition = leftSlide.getCurrentPosition();

        double startTime = getRuntime();
        double velocity = 0;

        while (opModeIsActive()) {
            double elapsedTime = getRuntime() - startTime;

            // Calculate desired velocity based on motion profiling
            double distanceToTarget = targetPosition - currentRightPosition;
            if (distanceToTarget > 0) {
                // Accelerate to max velocity, then decelerate near the target
                velocity = Math.min(MAX_ACCELERATION_TICKS * elapsedTime,
                        Math.sqrt(2 * MAX_ACCELERATION_TICKS * distanceToTarget));
                velocity = Math.min(velocity, MAX_VELOCITY_TICKS);
            } else {
                velocity = 0; // Stop if we reach the target
            }

            // Update motor powers proportionally
            double power = velocity / MAX_VELOCITY_TICKS;

            rightSlide.setVelocity(velocity);
            leftSlide.setVelocity(velocity);

            // Update telemetry
            currentRightPosition = rightSlide.getCurrentPosition();
            currentLeftPosition = leftSlide.getCurrentPosition();
            telemetry.addData("Right Position", currentRightPosition);
            telemetry.addData("Left Position", currentLeftPosition);
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Power", power);
            telemetry.update();

            // Stop if target is reached
            if (Math.abs(distanceToTarget) < 10) {
                break;
            }
        }
    }
}

