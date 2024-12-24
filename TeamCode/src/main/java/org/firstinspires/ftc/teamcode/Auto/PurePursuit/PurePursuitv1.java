package org.firstinspires.ftc.teamcode.Auto.PurePursuit;

import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.PathingAlgorithm;
import org.firstinspires.ftc.teamcode.ModulesToImport.Systems.Robot;
import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.SplineGenerator;
import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.TrajectoryAction;
import org.firstinspires.ftc.teamcode.ModulesToImport.Systems.Lifts;

import java.util.List;
import java.util.ArrayList;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous(name = "PurePursiutv1", group = "Autonomous")
public class PurePursuitv1 extends LinearOpMode {
    Robot robot = new Robot();
    //Lifts lifts = new Lifts(hardwareMap);

    @Override
    public void runOpMode() {
        // Initialize hardware and odometry
        double ticksPerInch = 537.6 / (Math.PI * 4.0); // Example for GoBILDA motor with 4-inch wheels
        double wheelBaseWidth = 14.0; // Example distance between left and right wheels in inches
        double horizontalOffset = -3.5; // Example distance of horizontal wheel from center
        robot.init(hardwareMap, ticksPerInch, wheelBaseWidth, horizontalOffset);

        // Define waypoints
        List<PathingAlgorithm.Point> waypoints = new ArrayList<>();
        waypoints.add(PathingAlgorithm.Point.toGridIndex(-70, -70));
        waypoints.add(PathingAlgorithm.Point.toGridIndex(0, 0));
        waypoints.add(PathingAlgorithm.Point.toGridIndex(70, 70));

        // Define trajectory actions
        //List<TrajectoryAction> actions = new ArrayList<>();
        //actions.add(new TrajectoryAction(0, 0, () -> moveSlidesUp()));
        //actions.add(new TrajectoryAction(70, 70, () -> moveSlidesDown()));

        // Generate spline trajectory
        List<SplineGenerator.SplineSegment> spline = SplineGenerator.generateSpline(waypoints);

        waitForStart();

        // Follow spline trajectory
        for (SplineGenerator.SplineSegment segment : spline) {
            for (double t = 0; t <= 1; t += 0.05) {
                if (!opModeIsActive()) return;

                // Spline evaluation
                double[] position = segment.evaluate(t);
                double targetX = position[0];
                double targetY = position[1];
                double targetHeading = position[2];

                // Update odometry
                robot.updateOdometry();
                double currentX = robot.getX();
                double currentY = robot.getY();

                // Execute actions
                /*for (TrajectoryAction action : actions) {
                    if (action.isConditionMet(currentX, currentY, 1.0)) {
                        action.execute();
                    }
                }*/

                // Calculate motor power
                double forward = calculateForwardVelocity(targetX - currentX, targetY - currentY);
                double strafe = calculateStrafeVelocity(targetX - currentX, targetY - currentY);
                double rotate = calculateHeadingCorrection(targetHeading - robot.getHeading());
                double acceleration = calculateAcceleration(forward, strafe);

                // Apply power
                robot.setTargetPose(60,60,Math.toRadians(60));
                sleep(50);
            }
        }

        robot.setTargetPose(0, 0, 0);
    }

    private void moveSlidesUp() {
        telemetry.addLine("Moving slides up!");
        telemetry.update();
        //lifts.setTargetPosition(3000);

    }

    private void moveSlidesDown() {
        telemetry.addLine("Moving slides down!");
        telemetry.update();
        //lifts.setTargetPosition(0);
        sleep(500);
    }

    private double calculateForwardVelocity(double deltaX, double deltaY) {
        return 0.1 * deltaY;
    }

    private double calculateStrafeVelocity(double deltaX, double deltaY) {
        return 0.1 * deltaX;
    }

    private double calculateHeadingCorrection(double headingError) {
        return 0.1 * headingError;
    }

    private double calculateAcceleration(double forward, double strafe) {
        double velocity = Math.sqrt(forward * forward + strafe * strafe);
        return 0.2 * velocity;
    }
}