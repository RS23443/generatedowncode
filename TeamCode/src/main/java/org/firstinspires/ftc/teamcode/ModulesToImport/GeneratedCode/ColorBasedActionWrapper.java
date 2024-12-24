package org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

public class ColorBasedActionWrapper {
    // A method to wrap a single trajectory action with a color condition
    public static Action wrapWithColorCheck(Action trajectory, ColorSensor leftSensor, ColorSensor rightSensor, ColorCondition condition, String blueOrRed, MecanumDrive drivetrain) {
        return new Action() {
            private final Action wrappedAction = trajectory; // The trajectory to be wrapped

            @Override
            public boolean run(TelemetryPacket packet) {
                // Check the color condition
                if (condition.shouldStop(leftSensor, rightSensor, blueOrRed)) {
                    // Stop the trajectory
                    drivetrain.leftFront.setPower(0);
                    drivetrain.leftBack.setPower(0);
                    drivetrain.rightBack.setPower(0);
                    drivetrain.rightFront.setPower(0);
                    // Stop the robot motors or call necessary halt methods
                    // Ensure you have access to the drive system to stop motors here

                    return false; // End the action
                }

                // Continue running the wrapped action if condition not met
                return wrappedAction.run(packet);
            }

            @Override
            public void preview(Canvas canvas) {
                // Preview the original trajectory
                wrappedAction.preview(canvas);
            }
        };
    }

    // Generate a list of wrapped actions from a list of trajectories
    public static List<Action> wrapAllWithColorCheck(List<Action> trajectories, ColorSensor leftSensor, ColorSensor rightSensor, ColorCondition condition, String blueOrRed, MecanumDrive drivetrain) {
        List<Action> wrappedActions = new ArrayList<>();
        for (Action trajectory : trajectories) {
            wrappedActions.add(wrapWithColorCheck(trajectory, leftSensor, rightSensor, condition, blueOrRed, drivetrain));
        }
        return wrappedActions;
    }



    // Interface for defining a color condition
    public interface ColorCondition {
            boolean shouldStop(ColorSensor leftSensor, ColorSensor rightSensor, String blueOrRed);

    }
}
