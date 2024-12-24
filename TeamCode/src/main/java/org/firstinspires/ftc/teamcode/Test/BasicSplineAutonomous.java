package org.firstinspires.ftc.teamcode.Test;

import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.PathingAlgorithm;
import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.SplineGenerator;
import org.firstinspires.ftc.teamcode.ModulesToImport.Systems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;
@Disabled
@Autonomous(name = "BasicSplineAutonomous", group = "Test")
public class BasicSplineAutonomous extends LinearOpMode {
    private Robot robot = new Robot();

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Initialize robot and odometry
        double ticksPerInch = 537.6 / (Math.PI * 4.0); // Example for 4-inch wheel
        double wheelBaseWidth = 9.5; // Distance between left and right wheels in inches
        double horizontalOffset = -3.5; // Horizontal offset in inches
        robot.init(hardwareMap, ticksPerInch, wheelBaseWidth, horizontalOffset);
        robot.resetOdometryPose(-70,-70,0);
        // Define waypoints
        List<PathingAlgorithm.Point> waypoints = new ArrayList<>();
        waypoints.add(new PathingAlgorithm.Point(-70, -70, Math.toRadians(0)));   // Start point
        waypoints.add(new PathingAlgorithm.Point(-50, -50, Math.toRadians(45))); // Intermediate
        waypoints.add(new PathingAlgorithm.Point(-30, -30, Math.toRadians(90))); // Target point

        // Generate spline
        List<SplineGenerator.SplineSegment> spline = SplineGenerator.generateSpline(waypoints);

        telemetry.addLine("Initialization complete. Waiting for start...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addLine("Starting trajectory...");
            telemetry.update();

            // Loop through the spline and follow the trajectory
            for (SplineGenerator.SplineSegment segment : spline) {
                for (double t = 0; t <= 1; t += 0.01) {
                    if (!opModeIsActive()) break;

                    //robot.updateOdometrysmiple();
                    //robot.updateOdometry();

                    // Evaluate the spline to get the target pose
                    double[] targetPose = segment.evaluate(t);
                    double targetX = targetPose[0];
                    double targetY = targetPose[1];
                    double targetHeading = targetPose[2];

                    // Get current pose from odometry
                    double currentX = robot.getX();
                    double currentY = robot.getY();
                    double currentHeading = robot.getHeading();

                    // Calculate errors
                    double errorX = targetX - currentX;
                    double errorY = targetY - currentY;
                    double errorHeading = targetHeading - currentHeading;

                    // Apply control logic to minimize the errors
                    double forward = calculateForwardVelocity(errorX, errorY);
                    double strafe = calculateStrafeVelocity(errorX, errorY);
                    double rotate = calculateHeadingCorrection(errorHeading);

                    // Send power to drivetrain
                    robot.updateDrivetrain(forward, strafe, rotate, 0.2, 0.2, 0.2); // Example accelerations

                    // Debugging telemetry
                    telemetry.addData("Current Pose", "X: %.2f, Y: %.2f, Heading: %.2f deg", currentX, currentY, Math.toDegrees(currentHeading));
                    telemetry.addData("Target Pose", "X: %.2f, Y: %.2f, Heading: %.2f deg", targetX, targetY, Math.toDegrees(targetHeading));
                    telemetry.update();

                    sleep(20); // Short delay for control loop
                    robot.updateOdometrysmiple();
                }
            }

            telemetry.addLine("Trajectory complete. Stopping robot.");
            telemetry.update();
            robot.stopMotors();
        }
    }

    // Calculate forward velocity (example proportional control)
    private double calculateForwardVelocity(double errorX, double errorY) {
        return 0.5 * errorY; // Modify the constant (0.1) as needed
    }

    // Calculate strafe velocity (example proportional control)
    private double calculateStrafeVelocity(double errorX, double errorY) {
        return 0.5 * errorX; // Modify the constant (0.1) as needed
    }

    // Calculate heading correction (example proportional control)
    private double calculateHeadingCorrection(double errorHeading) {
        return 0.5 * errorHeading; // Modify the constant (0.1) as needed
    }
}
