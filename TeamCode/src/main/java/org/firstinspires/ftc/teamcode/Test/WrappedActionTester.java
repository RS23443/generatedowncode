package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.ColorBasedActionWrapper;
import org.firstinspires.ftc.teamcode.ModulesToImport.Conditionals.RedOrBlueCondition;

@Autonomous(name = "Wrapped Action Tester", group = "Test")
public class WrappedActionTester extends LinearOpMode {
    Pose2d initialpose = new Pose2d(0,0,Math.toRadians(180));
    @Override
    public void runOpMode() {
        // Initialize drivetrain
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, initialpose);

        // Initialize color sensors
        ColorSensor leftSensor = hardwareMap.get(ColorSensor.class, "leftSensor");
        ColorSensor rightSensor = hardwareMap.get(ColorSensor.class, "rightSensor");

        // Define a simple trajectory
        TrajectoryActionBuilder trajectory = drivetrain.actionBuilder(initialpose)
                .setTangent(0)
                .lineToXLinearHeading(24,Math.toRadians(180))
                .endTrajectory();


        // Wrap the trajectory with the color condition
        Action wrappedTrajectory = ColorBasedActionWrapper.wrapWithColorCheck(
                trajectory.build(),  // The original trajectory
                leftSensor,          // Left color sensor
                rightSensor,         // Right color sensor
                new RedOrBlueCondition(),           // Color condition
                "Red",               // Target color (not strictly needed in this simple example)
                drivetrain           // Drivetrain for stopping
        );

        // Wait for the driver to start the autonomous routine
        waitForStart();
            // Execute the wrapped trajectory
            Actions.runBlocking(wrappedTrajectory);
    }
}
