package org.firstinspires.ftc.teamcode.ModulesToImport.systems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lifts {
    private DcMotorEx leftLiftMotor;
    private DcMotorEx rightLiftMotor;

    // PIDF coefficients
    private static final double kP = 0.01; // Proportional constant
    private static final double kI = 0.0;  // Integral constant
    private static final double kD = 0.0;  // Derivative constant
    private static final double kF = 0.0005; // Feedforward constant

    private static final double MAX_POWER = 1.0; // Maximum motor power
    private static final double MIN_POWER = 0.1; // Minimum power to overcome friction

    private int targetPosition; // Target encoder position
    private double targetVelocity; // Target velocity (if needed for feedforward)
    private boolean isMoving = false; // Whether the lift is currently moving

    private double previousError = 0.0; // For calculating D (derivative) term
    private double integralSum = 0.0; // For calculating I (integral) term

    public Lifts(HardwareMap hardwareMap) {
        // Initialize motors
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightslide");

        // Reverse one motor if needed
        rightLiftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Reset encoders
        resetEncoders();

        // Set motors to run using encoders
        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set brake behavior when power is 0
        leftLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    // Reset encoders
    public void resetEncoders() {
        leftLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Set the target position for the lift
    public void setTargetPosition(int position) {
        targetPosition = position;
        isMoving = true;
        integralSum = 0.0; // Reset integral term
        previousError = 0.0; // Reset derivative term
    }

    // Update the lift's position and power
    public void update() {
        if (isMoving) {
            // Get current positions
            int currentPositionLeft = leftLiftMotor.getCurrentPosition();
            int currentPositionRight = rightLiftMotor.getCurrentPosition();
            int currentPosition = (currentPositionLeft + currentPositionRight) / 2;

            // Calculate error
            int error = targetPosition - currentPosition;

            // Calculate P, I, and D terms
            double proportional = kP * error;
            integralSum += error; // Accumulate error over time
            double integral = kI * integralSum;
            double derivative = kD * (error - previousError);
            previousError = error;

            // Calculate feedforward term
            double feedforward = kF * targetPosition; // Proportional to target position

            // Combine PIDF terms
            double power = proportional + integral + derivative + feedforward;

            // Clamp power values to avoid exceeding motor limits
            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

            // Ensure minimum power to overcome static friction
            if (Math.abs(power) < MIN_POWER) {
                power = Math.signum(power) * MIN_POWER;
            }

            // Apply power to motors
            leftLiftMotor.setPower(power);
            rightLiftMotor.setPower(power);

            // Stop motors if within threshold
            if (Math.abs(error) < 10) { // Threshold = 10 ticks
                stop();
            }
        }
    }

    // Stop the lift
    public void stop() {
        leftLiftMotor.setPower(0);
        rightLiftMotor.setPower(0);
        isMoving = false;
    }

    // Check if the lift is moving
    public boolean isMoving() {
        return isMoving;
    }

    // Get current position (average of both motors)
    public int getCurrentPosition() {
        return (leftLiftMotor.getCurrentPosition() + rightLiftMotor.getCurrentPosition()) / 2;
    }
}





