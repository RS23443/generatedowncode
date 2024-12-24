package org.firstinspires.ftc.teamcode.Auto.enumbasedcode;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class EnumIntake extends SubsystemBase {

    // Enums for defining positions
    public enum IntakeExtensionPosition {
        MAX_EXTENSION,
        THIRD_MAX_EXTENSION,
        TRANSFER
    }

    public enum IntakeSpinPosition {
        VERTICAL,
        HORIZONTAL
    }

    public enum ClawPosition {
        OPEN,
        CLOSE
    }

    public enum IntakeFlip {
        UP,
        DOWN,
        TRANSFER
    }

    // Servos for the subsystem
    private final Servo lif;
    private final Servo rif;
    private final Servo rext;
    private final Servo lext;
    private final Servo intake;
    private final Servo intakespin;

    // Constructor
    public EnumIntake(final HardwareMap hardwareMap, final String lifname, final String rifname,
                      final String lextname, final String rextname, final String intakeclawname,
                      final String intakespinname) {
        lif = hardwareMap.get(Servo.class, lifname);
        rif = hardwareMap.get(Servo.class, rifname);
        lext = hardwareMap.get(Servo.class, lextname);
        rext = hardwareMap.get(Servo.class, rextname);
        intake = hardwareMap.get(Servo.class, intakeclawname);
        intakespin = hardwareMap.get(Servo.class, intakespinname);
    }

    // Set positions using enums
    public void setClawPosition(ClawPosition position) {
        switch (position) {
            case OPEN:
                intake.setPosition(0.3);
                break;
            case CLOSE:
                intake.setPosition(0.0);
                break;
        }
    }

    public void setIntakeSpinPosition(IntakeSpinPosition position) {
        switch (position) {
            case VERTICAL:
                intakespin.setPosition(0.6);
                break;
            case HORIZONTAL:
                intakespin.setPosition(0.1);
                break;
        }
    }

    public void setIntakeFlipPosition(IntakeFlip position) {
        switch (position) {
            case UP:
                lif.setPosition(0.95);
                rif.setPosition(0.05);
                break;
            case DOWN:
                lif.setPosition(0.15);
                rif.setPosition(0.85);
                break;
            case TRANSFER:
                lif.setPosition(0.95);
                rif.setPosition(0.05);
                break;
        }
    }

    public void setExtensionPosition(IntakeExtensionPosition position) {
        switch (position) {
            case MAX_EXTENSION:
                rext.setPosition(0.8);
                lext.setPosition(0.10);
                break;
            case THIRD_MAX_EXTENSION:
                rext.setPosition(0.8);
                lext.setPosition(0.10);
                break;
            case TRANSFER:
                rext.setPosition(0.5);
                lext.setPosition(0.4);
                break;
        }
    }

    // IntakeSetPositionAction for setting multiple positions at once
    public class IntakeSetPositionAction implements Action {
        private final IntakeExtensionPosition extPos;
        private final IntakeSpinPosition spinPos;
        private final ClawPosition clawPos;
        private final IntakeFlip flipPos;
        private final long delay; // Delay in milliseconds

        public IntakeSetPositionAction(IntakeExtensionPosition extPos, IntakeSpinPosition spinPos,
                                       ClawPosition clawPos, IntakeFlip flipPos, long delay) {
            this.extPos = extPos;
            this.spinPos = spinPos;
            this.clawPos = clawPos;
            this.flipPos = flipPos;
            this.delay = delay;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            try {
                setClawPosition(clawPos);
                Thread.sleep(delay / 4); // Dividing delay equally among operations
                setIntakeSpinPosition(spinPos);
                Thread.sleep(delay / 4);
                setIntakeFlipPosition(flipPos);
                Thread.sleep(delay / 4);
                setExtensionPosition(extPos);
                Thread.sleep(delay / 4);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            return false; // Action completes immediately after execution
        }
    }

    public Action intakeSetPositionAction(IntakeExtensionPosition extPos, IntakeSpinPosition spinPos,
                                          ClawPosition clawPos, IntakeFlip flipPos, long delay) {
        return new IntakeSetPositionAction(extPos, spinPos, clawPos, flipPos, delay);
    }

    public class SetClawAction implements Action {
        private final ClawPosition position;

        public SetClawAction(ClawPosition position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setClawPosition(position);
            return false; // Action completes immediately
        }
    }

    public Action setClawAction(ClawPosition position) {
        return new SetClawAction(position);
    }

    public class SetSpinAction implements Action {
        private final IntakeSpinPosition position;

        public SetSpinAction(IntakeSpinPosition position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setIntakeSpinPosition(position);
            return false; // Action completes immediately
        }
    }

    public Action setSpinAction(IntakeSpinPosition position) {
        return new SetSpinAction(position);
    }

    public class SetFlipAction implements Action {
        private final IntakeFlip position;

        public SetFlipAction(IntakeFlip position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setIntakeFlipPosition(position);
            return false; // Action completes immediately
        }
    }

    public Action setFlipAction(IntakeFlip position) {
        return new SetFlipAction(position);
    }

    public class SetExtensionAction implements Action {
        private final IntakeExtensionPosition position;

        public SetExtensionAction(IntakeExtensionPosition position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setExtensionPosition(position);
            return false; // Action completes immediately
        }
    }

    public Action setExtensionAction(IntakeExtensionPosition position) {
        return new SetExtensionAction(position);
    }
}
