package org.firstinspires.ftc.teamcode.ModulesToImport.Systems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;

public class Robot {
    // Drivetrain motors
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    // Odometry system
    private Odometry odometry;

    // PID Controllers
    private PIDController lateralPID; // X-direction
    private PIDController verticalPID; // Y-direction
    private PIDController angularPID; // Heading

    // Feedforward coefficients
    private static final double kV = 0.00032;   // Velocity coefficient
    private static final double kA = 0.00006; // Acceleration coefficient
    private static final double kS = 1.4257472641663829;  // Static friction coefficient

    public void init(HardwareMap hardwareMap, double ticksPerInch, double wheelBaseWidth, double horizontalOffset) {
        // Initialize drivetrain motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize odometry system
        odometry = new Odometry(hardwareMap, ticksPerInch, wheelBaseWidth, horizontalOffset);

        // Initialize PID controllers
        lateralPID = new PIDController(6.0, 0.05, 6.00); // Adjust gains for X direction
        verticalPID = new PIDController(6.0, 0.05, 4.00); // Adjust gains for Y direction
        angularPID = new PIDController(6.0, 0.05, 0.00); // Adjust gains for heading
    }

    // Update odometry system
    public void updateOdometry() {
        odometry.update();
    }
    public void updateOdometrysmiple() {
        odometry.updatesmiple();
    }

    // Reset odometry pose
    public void resetOdometryPose(double startX, double startY, double startHeading) {
        odometry.resetPose(startX, startY, startHeading);
    }

    public int[] getPosition() {
        return odometry.getPosition();
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

    // Set target positions with PID
    public void setTargetPose(double targetX, double targetY, double targetHeading) {
        lateralPID.setSetPoint(targetX); // Target X in inches
        verticalPID.setSetPoint(targetY); // Target Y in inches
        angularPID.setSetPoint(targetHeading); // Target heading in radians
    }

    // Update drivetrain with Feedforward + PID
    public void updateDrivetrain(double desiredVelocityX, double desiredVelocityY, double desiredAngularVelocity,
                                 double accelerationX, double accelerationY, double angularAcceleration) {
        // Get current state
        double currentX = getX();
        double currentY = getY();
        double currentHeading = getHeading();

        // Calculate PID outputs
        double lateralPIDOutput = lateralPID.calculate(currentX); // X-axis position correction
        double verticalPIDOutput = verticalPID.calculate(currentY); // Y-axis position correction
        double angularPIDOutput = angularPID.calculate(currentHeading); // Heading correction

        // Calculate Feedforward outputs
        double lateralFFOutput = calculateFeedforward(desiredVelocityX, accelerationX);
        double verticalFFOutput = calculateFeedforward(desiredVelocityY, accelerationY);
        double angularFFOutput = calculateFeedforward(desiredAngularVelocity, angularAcceleration);

        // Combine PID and Feedforward
        double lateralPower = lateralPIDOutput + lateralFFOutput;
        double verticalPower = verticalPIDOutput + verticalFFOutput;
        double angularPower = angularPIDOutput + angularFFOutput;

        // Cap motor speeds
        lateralPower = Math.max(-1.0, Math.min(1.0, lateralPower));
        verticalPower = Math.max(-1.0, Math.min(1.0, verticalPower));
        angularPower = Math.max(-1.0, Math.min(1.0, angularPower));

        // Calculate motor powers for mecanum drive
        double frontLeftPower = lateralPower + verticalPower + angularPower;
        double frontRightPower = lateralPower - verticalPower - angularPower;
        double backLeftPower = lateralPower - verticalPower + angularPower;
        double backRightPower = lateralPower + verticalPower - angularPower;

        // Normalize powers
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

    // Stop all motors
    public void stopMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
