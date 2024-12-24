package org.firstinspires.ftc.teamcode.Auto.enumbasedcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class Enum_Robot_Controller extends SubsystemBase {

    private final EnumIntake intake;
    private final EnumDeposit depo;

    public Enum_Robot_Controller(EnumIntake intake, EnumDeposit depo) {
        this.intake = intake;
        this.depo = depo;
    }

    // ** Deposit actions **
    public class DepoSetPositionAction implements Action {
        private final EnumDeposit.DepositFlipPosition flipPos;
        private final EnumDeposit.DepositOutPosition outPos;
        private final EnumDeposit.FingerPosition fingerPos;
        private final EnumDeposit.SpinPosition spinPos;
        private final long delay;

        public DepoSetPositionAction(EnumDeposit.DepositFlipPosition flipPos, EnumDeposit.DepositOutPosition outPos,
                                     EnumDeposit.FingerPosition fingerPos, EnumDeposit.SpinPosition spinPos, long delay) {
            this.flipPos = flipPos;
            this.outPos = outPos;
            this.fingerPos = fingerPos;
            this.spinPos = spinPos;
            this.delay = delay;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            try {
                depo.setFingerPosition(fingerPos);
                Thread.sleep(delay / 4);
                depo.setDepositFlipPosition(flipPos);
                Thread.sleep(delay / 4);
                depo.setOutPosition(outPos);
                Thread.sleep(delay / 4);
                depo.setSpinPosition(spinPos);
                Thread.sleep(delay / 4);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            return false;
        }
    }

    public Action depoSetPosition(EnumDeposit.DepositFlipPosition flipPos, EnumDeposit.DepositOutPosition outPos,
                                  EnumDeposit.FingerPosition fingerPos, EnumDeposit.SpinPosition spinPos, long delay) {
        return new DepoSetPositionAction(flipPos, outPos, fingerPos, spinPos, delay);
    }

    public Action depoGrabPose() {
        return depo.grabPoseAction();
    }

    public Action depoDumpPose() {
        return depo.dropPoseAction();
    }

    // ** Intake actions **
    public class IntakeSetPositionAction implements Action {
        private final EnumIntake.IntakeExtensionPosition extPos;
        private final EnumIntake.IntakeSpinPosition spinPos;
        private final EnumIntake.ClawPosition clawPos;
        private final EnumIntake.IntakeFlip flipPos;
        private final long delay;

        public IntakeSetPositionAction(EnumIntake.IntakeExtensionPosition extPos, EnumIntake.IntakeSpinPosition spinPos,
                                       EnumIntake.ClawPosition clawPos, EnumIntake.IntakeFlip flipPos, long delay) {
            this.extPos = extPos;
            this.spinPos = spinPos;
            this.clawPos = clawPos;
            this.flipPos = flipPos;
            this.delay = delay;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            try {
                intake.setClawPosition(clawPos);
                Thread.sleep(delay / 4);
                intake.setIntakeSpinPosition(spinPos);
                Thread.sleep(delay / 4);
                intake.setIntakeFlipPosition(flipPos);
                Thread.sleep(delay / 4);
                intake.setExtensionPosition(extPos);
                Thread.sleep(delay / 4);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            return false;
        }
    }

    public Action intakeSetPosition(EnumIntake.IntakeExtensionPosition extPos, EnumIntake.IntakeSpinPosition spinPos,
                                    EnumIntake.ClawPosition clawPos, EnumIntake.IntakeFlip flipPos, long delay) {
        return new IntakeSetPositionAction(extPos, spinPos, clawPos, flipPos, delay);
    }

    // Convenience actions for common operations
    public Action intakeMaxExtension() {
        return intake.intakeSetPositionAction(
                EnumIntake.IntakeExtensionPosition.MAX_EXTENSION,
                EnumIntake.IntakeSpinPosition.HORIZONTAL,
                EnumIntake.ClawPosition.CLOSE,
                EnumIntake.IntakeFlip.UP,
                200
        );
    }

    public Action intakeTransferPose() {
        return intake.intakeSetPositionAction(
                EnumIntake.IntakeExtensionPosition.TRANSFER,
                EnumIntake.IntakeSpinPosition.HORIZONTAL,
                EnumIntake.ClawPosition.OPEN,
                EnumIntake.IntakeFlip.TRANSFER,
                200
        );
    }



    public Action intakeDrop() {
        return intake.intakeSetPositionAction(
                EnumIntake.IntakeExtensionPosition.THIRD_MAX_EXTENSION,
                EnumIntake.IntakeSpinPosition.VERTICAL,
                EnumIntake.ClawPosition.CLOSE,
                EnumIntake.IntakeFlip.DOWN,
                200
        );
    }

    // ** Transfer action from intake to deposit **
    public Action transferToDeposit() {
        return new Action() {
            @Override
            public boolean run(TelemetryPacket packet) {
                try {
                    depo.setDepositFlipPosition(EnumDeposit.DepositFlipPosition.GRAB);
                    Thread.sleep(200);

                    depo.setFingerPosition(EnumDeposit.FingerPosition.CLOSE);
                    Thread.sleep(200);

                    intake.setClawPosition(EnumIntake.ClawPosition.OPEN);
                    depo.setDepositFlipPosition(EnumDeposit.DepositFlipPosition.HALF_POSE);

                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                return false; // Action completed
            }
        };
    }

    // ** Full Transfer to Deposit Action **
    public class FullTransferToDepositAction implements Action {
        private final EnumIntake.IntakeExtensionPosition intakeExtPos;
        private final EnumIntake.ClawPosition intakeClawPos1;
        private final EnumIntake.ClawPosition intakeClawPos2;
        private final EnumDeposit.DepositFlipPosition depoGrabPos;
        private final EnumDeposit.FingerPosition depoFingerPos;
        private final EnumDeposit.DepositFlipPosition depoHalfPose;
        private final long delay;

        public FullTransferToDepositAction(EnumIntake.ClawPosition intakeClawPos1,
                                           EnumIntake.IntakeExtensionPosition intakeExtPos,
                                           EnumIntake.ClawPosition intakeClawPos2,
                                           EnumDeposit.DepositFlipPosition depoGrabPos,
                                           EnumDeposit.FingerPosition depoFingerPos,
                                           EnumDeposit.DepositFlipPosition depoHalfPose,
                                           long delay) {
            this.intakeExtPos = intakeExtPos;
            this.intakeClawPos1 = intakeClawPos1;
            this.intakeClawPos2 = intakeClawPos2;
            this.depoGrabPos = depoGrabPos;
            this.depoFingerPos = depoFingerPos;
            this.depoHalfPose = depoHalfPose;
            this.delay = delay;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            try {
                intake.setClawPosition(intakeClawPos1);
                Thread.sleep(delay / 4);
                intake.setExtensionPosition(intakeExtPos);
                Thread.sleep(delay / 4);
                depo.setDepositFlipPosition(depoGrabPos);
                Thread.sleep(delay / 4);
                depo.setFingerPosition(depoFingerPos);
                Thread.sleep(delay / 4);
                intake.setClawPosition(intakeClawPos2);
                Thread.sleep(delay / 4);
                depo.setDepositFlipPosition(depoHalfPose);
                Thread.sleep(delay / 4);

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            return false;
        }
    }

    public Action fullTransferToDeposit(EnumIntake.IntakeExtensionPosition intakeExtPos,
                                        EnumIntake.ClawPosition intakeClawPos1,
                                        EnumIntake.ClawPosition intakeClawPos2,
                                        EnumDeposit.DepositFlipPosition depoGrabPos,
                                        EnumDeposit.FingerPosition depoFingerPos,
                                        EnumDeposit.DepositFlipPosition depoHalfPose,
                                        long delay) {
        return new FullTransferToDepositAction(intakeClawPos1, intakeExtPos, intakeClawPos2, depoGrabPos, depoFingerPos, depoHalfPose, delay);
    }

    // ** Dump, Open, and Reset Action **
    public class DumpOpenAndResetAction implements Action {
        private final EnumDeposit depo;
        private final EnumDeposit.DepositFlipPosition dumpPos;
        private final EnumDeposit.FingerPosition openPos;
        private final EnumDeposit.DepositFlipPosition grabPos;
        private final long delay;

        public DumpOpenAndResetAction(EnumDeposit depo,
                                      EnumDeposit.DepositFlipPosition dumpPos,
                                      EnumDeposit.FingerPosition openPos,
                                      EnumDeposit.DepositFlipPosition grabPos,
                                      long delay) {
            this.depo = depo;
            this.dumpPos = dumpPos;
            this.openPos = openPos;
            this.grabPos = grabPos;
            this.delay = delay;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            try {
                depo.setDepositFlipPosition(dumpPos);
                Thread.sleep(delay / 3);
                depo.setFingerPosition(openPos);
                Thread.sleep(delay / 3);
                depo.setDepositFlipPosition(grabPos);
                Thread.sleep(delay / 3);

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            return false;
        }
    }

    public Action dumpOpenAndReset(EnumDeposit.DepositFlipPosition dumpPos,
                                   EnumDeposit.FingerPosition openPos,
                                   EnumDeposit.DepositFlipPosition grabPos,
                                   long delay) {
        return new DumpOpenAndResetAction(depo, dumpPos, openPos, grabPos, delay);
    }
    // ** Close Intake Action **
    public class CloseIntakeAction implements Action {
        private final EnumIntake intake;
        private final EnumIntake.ClawPosition closePos;
        private final long delay;

        public CloseIntakeAction(EnumIntake intake, EnumIntake.ClawPosition closePos, long delay) {
            this.intake = intake;
            this.closePos = closePos;
            this.delay = delay;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            try {
                // Close the intake claw
                intake.setClawPosition(closePos);
                Thread.sleep(delay);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            return false;
        }
    }

    // Method to create a CloseIntakeAction
    public Action closeIntake(long delay) {
        return new CloseIntakeAction(intake, EnumIntake.ClawPosition.CLOSE, delay);
    }

}
