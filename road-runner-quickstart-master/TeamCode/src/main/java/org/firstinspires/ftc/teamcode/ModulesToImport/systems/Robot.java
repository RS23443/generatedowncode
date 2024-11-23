package org.firstinspires.ftc.teamcode.ModulesToImport.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    // Drivetrain motors
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    // Odometry system
    private Odometry odometry;

    // Motion constants
    private static final double MAX_VELOCITY = 60.0; // Inches per second
    private static final double MAX_ACCELERATION = 30.0; // Inches per second^2
    private static final double MAX_ANGULAR_VELOCITY = Math.toRadians(180.0); // Radians per second
    private static final double MAX_ANGULAR_ACCELERATION = Math.toRadians(90.0); // Radians per second^2

    // Feedforward coefficients (for advanced velocity control)
    private static final double kV = 0.01; // Velocity coefficient
    private static final double kA = 0.005; // Acceleration coefficient
    private static final double kS = 0.1; // Static friction coefficient

    public void init(HardwareMap hardwareMap, double ticksPerInch, double wheelBaseWidth, double horizontalOffset) {
        // Initialize drivetrain motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Initialize odometry system
        odometry = new Odometry(hardwareMap, ticksPerInch, wheelBaseWidth, horizontalOffset);
    }

    // Update odometry system
    public void updateOdometry() {
        odometry.update();
    }

    // Reset odometry pose
    public void resetOdometryPose(double startX, double startY, double startHeading) {
        odometry.resetPose(startX, startY, startHeading);
    }

    // Get robot pose
    public double getX() {
        return odometry.getX();
    }

    public double getY() {
        return odometry.getY();
    }

    public double getHeading() {
        return odometry.getHeading();
    }

    // Set motor power for Mecanum drive with feedforward control
    public void setMecanumPower(double forward, double strafe, double rotate, double acceleration) {
        double frontLeftPower = calculateFeedforward(forward + strafe + rotate, acceleration);
        double frontRightPower = calculateFeedforward(forward - strafe - rotate, acceleration);
        double backLeftPower = calculateFeedforward(forward - strafe + rotate, acceleration);
        double backRightPower = calculateFeedforward(forward + strafe - rotate, acceleration);

        // Normalize the power values to keep them within the [-1.0, 1.0] range
        double maxPower = Math.max(1.0, Math.max(
                Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        frontLeftPower /= maxPower;
        frontRightPower /= maxPower;
        backLeftPower /= maxPower;
        backRightPower /= maxPower;

        // Set motor powers
        leftFrontDrive.setPower(frontLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        leftBackDrive.setPower(backLeftPower);
        rightBackDrive.setPower(backRightPower);
    }

    // Calculate feedforward power
    private double calculateFeedforward(double velocity, double acceleration) {
        return kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
    }

    // Getters for motion constants
    public double getMaxVelocity() {
        return MAX_VELOCITY;
    }

    public double getMaxAcceleration() {
        return MAX_ACCELERATION;
    }

    public double getMaxAngularVelocity() {
        return MAX_ANGULAR_VELOCITY;
    }

    public double getMaxAngularAcceleration() {
        return MAX_ANGULAR_ACCELERATION;
    }
}


