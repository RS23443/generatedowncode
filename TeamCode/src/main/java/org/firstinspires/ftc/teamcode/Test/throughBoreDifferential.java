package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class throughBoreDifferential {
    private Encoder left, right;
    private final double ticksPerRotation = 8192;

    private int leftPreviousPosition = 0; // Previous left encoder position
    private int rightPreviousPosition = 0; // Previous right encoder position
    private int leftCurrentPosition = 0; // Current left encoder position
    private int rightCurrentPosition = 0; // Current right encoder position
    private int leftCumulativeDifference = 0; // Accumulated difference for the left encoder
    private int rightCumulativeDifference = 0; // Accumulated difference for the right encoder

    private static final int STOP_THRESHOLD = 400; // Max allowable cumulative difference
    private static final int SPINNING_THRESHOLD = 50; // Allowable difference for spinning (in ticks)

    public throughBoreDifferential(HardwareMap hardwareMap, String leftHook, String rightHook) {
        left = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, leftHook)));
        right = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, rightHook)));
    }

    // Updates current positions and accumulates the differences if not spinning
    public void updateCumulativeDifferences() {
        leftCurrentPosition = left.getPositionAndVelocity().rawPosition;
        rightCurrentPosition = right.getPositionAndVelocity().rawPosition;

        int leftDifference = leftCurrentPosition - leftPreviousPosition;
        int rightDifference = rightCurrentPosition - rightPreviousPosition;

        // Check if the differential is spinning
        if (!isSpinning(leftDifference, rightDifference)) {
            // Accumulate differences if not spinning
            leftCumulativeDifference += Math.abs(leftDifference);
            rightCumulativeDifference += Math.abs(rightDifference);
        }

        // Update previous positions
        leftPreviousPosition = leftCurrentPosition;
        rightPreviousPosition = rightCurrentPosition;
    }

    // Checks if the differential is spinning
    private boolean isSpinning(int leftDifference, int rightDifference) {
        return Math.abs(leftDifference - rightDifference) < SPINNING_THRESHOLD;
    }

    // Checks if the cumulative differences have exceeded the threshold
    public boolean shouldStop() {
        updateCumulativeDifferences();
        return leftCumulativeDifference >= STOP_THRESHOLD || rightCumulativeDifference >= STOP_THRESHOLD;
    }

    // Resets cumulative differences (e.g., after stopping or resetting)
    public void resetCumulativeDifferences() {
        leftCumulativeDifference = 0;
        rightCumulativeDifference = 0;
    }

    public void movementLimiter() {
        while (!shouldStop()) {
            updateCumulativeDifferences();
            adjustPower();
        }
        stopDifferential();
    }

    // Adjusts power to the differential servos (placeholder for actual servo logic)
    private void adjustPower() {
        if (rightCumulativeDifference > leftCumulativeDifference) {
            System.out.println("Adjusting differential: slowing down right side");
            // Adjust power for right servo
        } else {
            System.out.println("Adjusting differential: slowing down left side");
            // Adjust power for left servo
        }
    }

    // Stops all movement of the differential
    private void stopDifferential() {
        System.out.println("Stopping differential movement");
        // Add logic to stop servos
    }
}
