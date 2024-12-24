package org.firstinspires.ftc.teamcode.Auto.enumbasedcode;
import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class EnumDeposit extends SubsystemBase {

    // Enums for positions and actions
    public enum DepositFlipPosition {
        HALF_POSE,  // Used to be UP
        DUMP_OFF,   // Used to be DOWN
        GRAB        // Added new position
    }

    public enum DepositOutPosition {
        GRAB,   // Used to be PRELOAD
        DUMP    // Used to be TRANSFER
    }

    public enum FingerPosition {
        OPEN,
        CLOSE
    }

    public enum SpinPosition {
        SPIN_0,
        SPIN_90 // Updated value to spin(0.3)
    }

    // Servos for the deposit subsystem
    private final Servo lof;
    private final Servo rof;
    private final Servo outext;
    private final Servo spin;
    private final Servo finger;

    // Constructor
    public EnumDeposit(final HardwareMap hardwareMap, final String lofname, final String rofname,
                       final String outextname, final String spinname, final String fingername) {
        lof = hardwareMap.get(Servo.class, lofname);
        rof = hardwareMap.get(Servo.class, rofname);
        outext = hardwareMap.get(Servo.class, outextname);
        spin = hardwareMap.get(Servo.class, spinname);
        finger = hardwareMap.get(Servo.class, fingername);
    }

    // Set positions using enums
    public void setDepositFlipPosition(DepositFlipPosition position) {
        switch (position) {
            case HALF_POSE:
                lof.setPosition(0.35);
                rof.setPosition(0.65);
                break;
            case DUMP_OFF:
                lof.setPosition(0.75);
                rof.setPosition(0.25);
                break;
            case GRAB:
                lof.setPosition(0.315);
                rof.setPosition(0.685);
                break;
        }
    }

    public void setOutPosition(DepositOutPosition position) {
        switch (position) {
            case GRAB:
                outext.setPosition(0.5);
                break;
            case DUMP:
                outext.setPosition(0.0);
                break;
        }
    }

    public void setFingerPosition(FingerPosition position) {
        switch (position) {
            case OPEN:
                finger.setPosition(0.0);
                break;
            case CLOSE:
                finger.setPosition(0.67);
                break;
        }
    }

    public void setSpinPosition(SpinPosition position) {
        switch (position) {
            case SPIN_0:
                spin.setPosition(0.0);
                break;
            case SPIN_90:
                spin.setPosition(0.3); // Updated value
                break;
        }
    }

    // Actions for deposit movements
    public class DepositSetPositionAction implements Action {
        private final DepositFlipPosition flipPos;
        private final DepositOutPosition outPos;
        private final FingerPosition fingerPos;
        private final SpinPosition spinPos;
        private final long delay; // Delay in milliseconds

        public DepositSetPositionAction(DepositFlipPosition flipPos, DepositOutPosition outPos,
                                        FingerPosition fingerPos, SpinPosition spinPos, long delay) {
            this.flipPos = flipPos;
            this.outPos = outPos;
            this.fingerPos = fingerPos;
            this.spinPos = spinPos;
            this.delay = delay;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            try {
                setFingerPosition(fingerPos);
                Thread.sleep(delay / 4); // Dividing delay among operations
                setDepositFlipPosition(flipPos);
                Thread.sleep(delay / 4);
                setOutPosition(outPos);
                Thread.sleep(delay / 4);
                setSpinPosition(spinPos);
                Thread.sleep(delay / 4);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            return false; // Action completes immediately
        }
    }

    public Action depositSetPositionAction(DepositFlipPosition flipPos, DepositOutPosition outPos,
                                           FingerPosition fingerPos, SpinPosition spinPos, long delay) {
        return new DepositSetPositionAction(flipPos, outPos, fingerPos, spinPos, delay);
    }

    // Example specific actions for convenience
    public class GrabPoseAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            try {
                setFingerPosition(FingerPosition.OPEN);
                Thread.sleep(100);
                setDepositFlipPosition(DepositFlipPosition.GRAB);
                setOutPosition(DepositOutPosition.GRAB);
                Thread.sleep(100);
                setSpinPosition(SpinPosition.SPIN_0);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            return false;
        }
    }

    public Action grabPoseAction() {
        return new GrabPoseAction();
    }

    public class DropPoseAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            try {
                setFingerPosition(FingerPosition.CLOSE);
                Thread.sleep(100);
                setDepositFlipPosition(DepositFlipPosition.DUMP_OFF);
                setSpinPosition(SpinPosition.SPIN_0);
                Thread.sleep(100);
                setOutPosition(DepositOutPosition.DUMP);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            return false;
        }
    }

    public Action dropPoseAction() {
        return new DropPoseAction();
    }

}
