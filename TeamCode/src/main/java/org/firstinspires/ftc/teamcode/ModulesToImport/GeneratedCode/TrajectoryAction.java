package org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode;

public class TrajectoryAction {
    private final double targetX;
    private final double targetY;
    private final Runnable action; // Lambda or method reference to execute
    private boolean executed = false; // Prevent redundant execution

    public TrajectoryAction(double targetX, double targetY, Runnable action) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.action = action;
    }

    // Check if the robot is near the target point
    public boolean isConditionMet(double currentX, double currentY, double threshold) {
        return Math.abs(currentX - targetX) <= threshold && Math.abs(currentY - targetY) <= threshold;
    }

    // Execute the action
    public void execute() {
        if (!executed) { // Ensure the action runs only once
            action.run();
            executed = true;
        }
    }
}
