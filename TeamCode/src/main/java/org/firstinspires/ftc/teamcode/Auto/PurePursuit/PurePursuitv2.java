package org.firstinspires.ftc.teamcode.Auto.PurePursuit;

import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.PathingAlgorithm;
import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.SplineGenerator;
import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.TrajectoryAction;
import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.Timer;
import org.firstinspires.ftc.teamcode.ModulesToImport.Systems.Robot;
import org.firstinspires.ftc.teamcode.ModulesToImport.Systems.Lifts;
import org.firstinspires.ftc.teamcode.ModulesToImport.Systems.servoController;

import java.util.List;
import java.util.ArrayList;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
@Config
@Disabled
@Autonomous(name = "DhruvPursuitv2", group = "Autonomous")
public class PurePursuitv2 extends LinearOpMode {
    private Robot robot = new Robot();
    private Lifts lifts;
    private Timer timer = new Timer();
    private servoController servoController = new servoController();

    private String[] Servonames = {"left_intake_flip","left_extension","right_intake_flip","right_extension",
        "finger","outtake_ext","spin","left_outtake_flip","right_outtake_flip"};
    //intake servo is not being found: illeagl argumen exception
    private CRServo intake;


    // Time limit in milliseconds (30 seconds)
    private static final long MAX_RUNTIME_MILLIS = 30_000;

    @Override
    public void runOpMode() {
        telemetry.addLine("Starting initialization...");
        telemetry.update();

        // Initialize hardware and odometry
        double ticksPerInch = 537.6 / (Math.PI * 2.0* 1.8898); // calculates circumference of wheel
        double wheelBaseWidth = 9.5; // Distance between left and right wheels
        double horizontalOffset = -3.5; // Distance of horizontal wheel from center

        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        robot.init(hardwareMap, ticksPerInch, wheelBaseWidth, horizontalOffset);

        telemetry.addLine("Initializing Lifts...");
        telemetry.update();
        lifts = new Lifts(hardwareMap);

        telemetry.addLine("Initializing Servos...");
        telemetry.update();
        servoController.initl(hardwareMap, Servonames);

        intake = hardwareMap.get(CRServo.class, "intake");;

        // Define the starting pose (x, y, heading)
        int startX = -70;       // Start position in inches along X-axis
        int startY = -70;       // Start position in inches along Y-axis
        double startHeading = Math.toRadians(0); // Start facing forward

        telemetry.addLine("Setting starting pose...");
        telemetry.update();
        robot.resetOdometryPose(startX, startY, startHeading);

        // Define waypoints (10 waypoints with headings)
        telemetry.addLine("Defining waypoints...");
        telemetry.update();
        List<PathingAlgorithm.Point> waypoints = new ArrayList<>();
        waypoints.add(new PathingAlgorithm.Point(startX, startY, startHeading));   // Starting position
        waypoints.add(new PathingAlgorithm.Point(-70, -50, Math.toRadians(0))); // Facing 90 degrees
        //waypoints.add(new PathingAlgorithm.Point(-10, -10, Math.toRadians(90))); // Continue facing 90 degrees
        //waypoints.add(new PathingAlgorithm.Point(10, 10, Math.toRadians(180)));  // Reverse direction
        //waypoints.add(new PathingAlgorithm.Point(30, 30, Math.toRadians(270)));  // Turn to face downward
        //waypoints.add(new PathingAlgorithm.Point(50, 50, Math.toRadians(0)));    // Turn back to 0 degrees

        // Define trajectory actions
        telemetry.addLine("Defining trajectory actions...");
        telemetry.update();
        List<TrajectoryAction> actions = new ArrayList<>();
        actions.add(new TrajectoryAction(-10, -10, this::moveSlidesUp));
        actions.add(new TrajectoryAction(50, 50, this::moveSlidesDown));

        // Generate spline trajectory
        telemetry.addLine("Generating spline...");
        telemetry.update();
        List<SplineGenerator.SplineSegment> spline = SplineGenerator.generateSpline(waypoints);

        telemetry.addLine("Initialization complete! Waiting for start...");
        telemetry.update();
        waitForStart();

        // Start the timer
        telemetry.addLine("Starting timer...");
        telemetry.update();
        Timer.start();

        robot.resetOdometryPose(startX,startY,startHeading);
        telemetry.addLine("init pose has been set");
        telemetry.update();

        if (opModeIsActive()) {
            // Follow the spline trajectory
            for (SplineGenerator.SplineSegment segment : spline) {
                for (double t = 0; t <= 1; t += 0.05) {
                    if (!opModeIsActive() || Timer.getElapsedTimeMillis() > MAX_RUNTIME_MILLIS) {
                        shutdownSequence();
                        return;
                    }

                    // Spline evaluation
                    double[] position = segment.evaluate(t);
                    double targetX = position[0];
                    double targetY = position[1];
                    double targetHeading = position[2];

                    // Update odometry
                    robot.updateOdometry();
                    double currentX = robot.getX();
                    double currentY = robot.getY();
                    double currentHeading = robot.getHeading();

                    // Heading control
                    double rotate = calculateHeadingCorrection(targetHeading - currentHeading);

                    // Position control
                    double forward = calculateForwardVelocity(targetX - currentX, targetY - currentY);
                    double strafe = calculateStrafeVelocity(targetX - currentX, targetY - currentY);
                    double acceleration = calculateAcceleration(forward, strafe);

                    // Apply motor power
                    robot.updateDrivetrain(forward, strafe, rotate, acceleration, acceleration, acceleration);

                    // Telemetry for debugging
                    telemetry.addData("Current Pose", "X: %.2f, Y: %.2f, Heading: %.2f deg", currentX, currentY, Math.toDegrees(currentHeading));
                    telemetry.addData("Target Pose", "X: %.2f, Y: %.2f, Heading: %.2f deg", targetX, targetY, Math.toDegrees(targetHeading));
                    telemetry.addData("Elapsed Time", "%.2f seconds", Timer.getElapsedTimeMillis() / 1000.0);
                    telemetry.update();

                    sleep(20); // Short delay for control loop
                }
            }

            // Return to starting position (optional)
            telemetry.addLine("Returning to starting position...");
            telemetry.update();
            robot.setTargetPose(startX, startY, startHeading);
            robot.stopMotors();
        }
    }

    // Moves the lift to the "up" position
    private void moveSlidesUp() {
        telemetry.addLine("Executing: Moving slides up!");
        telemetry.update();
        lifts.setTargetPosition(3000);
        while (lifts.isMoving() && opModeIsActive()) {
            if (Timer.getElapsedTimeMillis() > MAX_RUNTIME_MILLIS) {
                shutdownSequence();
                return;
            }
            lifts.update();
        }
    }

    // Moves the lift to the "down" position
    private void moveSlidesDown() {
        telemetry.addLine("Executing: Moving slides down!");
        telemetry.update();
        lifts.setTargetPosition(0);
        while (lifts.isMoving() && opModeIsActive()) {
            if (Timer.getElapsedTimeMillis() > MAX_RUNTIME_MILLIS) {
                shutdownSequence();
                return;
            }
            lifts.update();
        }
    }

    // Shutdown sequence to ensure all motors stop
    private void shutdownSequence() {
        telemetry.addLine("Time limit reached! Shutting down...");
        telemetry.update();
        robot.stopMotors();
        lifts.stop();
    }

    // Calculates forward velocity
    private double calculateForwardVelocity(double deltaX, double deltaY) {
        return 0.1 * deltaY; // Modify as needed for robot's velocity model
    }

    // Calculates strafe velocity
    private double calculateStrafeVelocity(double deltaX, double deltaY) {
        return 0.1 * deltaX; // Modify as needed for robot's velocity model
    }

    // Calculates heading correction using proportional control
    private double calculateHeadingCorrection(double headingError) {
        return 0.1 * headingError; // Proportional heading correction
    }

    // Calculates acceleration
    private double calculateAcceleration(double forward, double strafe) {
        double velocity = Math.sqrt(forward * forward + strafe * strafe);
        return 0.2 * velocity; // Proportional acceleration
    }
}
