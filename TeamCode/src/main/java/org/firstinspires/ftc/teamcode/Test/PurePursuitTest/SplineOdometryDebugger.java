package org.firstinspires.ftc.teamcode.Test.PurePursuitTest;

import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.PathingAlgorithm;
import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.SplineGenerator;
import org.firstinspires.ftc.teamcode.ModulesToImport.Systems.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
@Disabled

@Autonomous(name = "SplineOdometryDebugger", group = "Test")
public class SplineOdometryDebugger extends LinearOpMode {
    private Robot robot = new Robot();
    private FileWriter writer;

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        // Initialize robot
        double ticksPerInch = 537.6 / (Math.PI * 4.0); // Example for 4-inch wheel
        double wheelBaseWidth = 9.5; // Wheelbase width in inches
        double horizontalOffset = -3.5; // Horizontal offset of the dead wheel
        robot.init(hardwareMap, ticksPerInch, wheelBaseWidth, horizontalOffset);

        // Define waypoints
        List<PathingAlgorithm.Point> waypoints = new ArrayList<>();
        waypoints.add(new PathingAlgorithm.Point(-70, -70, Math.toRadians(0)));   // Start point
        waypoints.add(new PathingAlgorithm.Point(-50, -50, Math.toRadians(45))); // Intermediate
        waypoints.add(new PathingAlgorithm.Point(-30, -30, Math.toRadians(90))); // Target point

        // Generate spline
        List<SplineGenerator.SplineSegment> spline = SplineGenerator.generateSpline(waypoints);

        // Prepare file for logging
        try {
            writer = new FileWriter("/sdcard/FIRST/spline_odometry_log.csv");
            writer.write("Time,OdometryX,OdometryY,OdometryHeading,TargetX,TargetY,TargetHeading,LeftEncoder,RightEncoder,HorizontalEncoder\n");
        } catch (IOException e) {
            telemetry.addLine("Failed to create log file.");
            telemetry.update();
        }

        telemetry.addLine("Initialization complete. Waiting for start...");
        telemetry.update();
        waitForStart();

        // Debugging loop
        long startTime = System.currentTimeMillis();
        for (SplineGenerator.SplineSegment segment : spline) {
            for (double t = 0; t <= 1; t += 0.05) {
                if (!opModeIsActive()) break;

                // Evaluate spline
                double[] position = segment.evaluate(t);
                double targetX = position[0];
                double targetY = position[1];
                double targetHeading = position[2];

                // Update odometry
                robot.updateOdometry();
                double currentX = robot.getX();
                double currentY = robot.getY();
                double currentHeading = robot.getHeading();

                // Retrieve encoder positions
                int[] encoders = robot.getPosition();
                int leftEncoder = encoders[0];
                int rightEncoder = encoders[1];
                int horizontalEncoder = encoders[2];

                // Log data to telemetry
                telemetry.addData("Time", (System.currentTimeMillis() - startTime) / 1000.0);
                telemetry.addData("Odometry", "X: %.2f, Y: %.2f, Heading: %.2f deg", currentX, currentY, Math.toDegrees(currentHeading));
                telemetry.addData("Target", "X: %.2f, Y: %.2f, Heading: %.2f deg", targetX, targetY, Math.toDegrees(targetHeading));
                telemetry.addData("Encoders", "Left: %d, Right: %d, Horizontal: %d", leftEncoder, rightEncoder, horizontalEncoder);
                telemetry.update();

                // Write to log file
                try {
                    writer.write(String.format("%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d\n",
                            (System.currentTimeMillis() - startTime),
                            currentX, currentY, Math.toDegrees(currentHeading),
                            targetX, targetY, Math.toDegrees(targetHeading),
                            leftEncoder, rightEncoder, horizontalEncoder));
                } catch (IOException e) {
                    telemetry.addLine("Error writing to log file.");
                    telemetry.update();
                }

                sleep(20); // Short delay
            }
        }

        // Close log file
        try {
            writer.close();
        } catch (IOException e) {
            telemetry.addLine("Error closing log file.");
            telemetry.update();
        }

        telemetry.addLine("Test complete. Logs saved.");
        telemetry.update();
    }
}
