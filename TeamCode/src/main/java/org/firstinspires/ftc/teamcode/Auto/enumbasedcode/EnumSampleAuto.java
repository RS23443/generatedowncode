package org.firstinspires.ftc.teamcode.Auto.enumbasedcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.lifts_subsystem;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@Autonomous(name = "4 Sample Auto - Enum Based", group = "Autonomous")
public class EnumSampleAuto extends LinearOpMode {
    public static final int LIFT_RETRACT_HEIGHT = 800; // PID height for retraction in ticks?
    public static final int LIFT_EXTEND_HEIGHT = 41;  // Non-PID height for extension (in inches)
    public static final double POWER_EXTEND = 1.0; //
    public static final long DELAY = 0; // in seconds

    @Override
    public void runOpMode() {
        // Define the initial pose
        Pose2d initialPose = new Pose2d(-14.5, -64, Math.toRadians(0));

        // Initialize drivetrain and subsystems
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, initialPose);
        EnumIntake intake = new EnumIntake(hardwareMap, "left_intake_flip", "right_intake_flip", "left_extension", "right_extension", "clawtakespin", "clawtake");
        EnumDeposit deposit = new EnumDeposit(hardwareMap, "left_outtake_flip", "right_outtake_flip", "outtake_ext", "spin", "finger");
        Enum_Robot_Controller robotController = new Enum_Robot_Controller(intake, deposit);
        lifts_subsystem lifts = new lifts_subsystem(hardwareMap,"leftslide","rightslide");

        // Wait for start
        waitForStart();

        if (isStopRequested()) return;

        // Define trajectories
        TrajectoryActionBuilder preload = drivetrain.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .lineToY(-56)
                .setTangent(0)
                .lineToXLinearHeading(-55, Math.toRadians(45.6557))
                .endTrajectory();

        // First sample pickup sequence
        TrajectoryActionBuilder pickSample1 = preload.endTrajectory().fresh()
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-58, -48, Math.toRadians(98)), 90)
                .stopAndAdd(robotController.intakeSetPosition(
                        EnumIntake.IntakeExtensionPosition.MAX_EXTENSION,
                        EnumIntake.IntakeSpinPosition.HORIZONTAL,
                        EnumIntake.ClawPosition.OPEN,
                        EnumIntake.IntakeFlip.DOWN,
                        500
                ))
                .stopAndAdd(robotController.fullTransferToDeposit(
                        EnumIntake.IntakeExtensionPosition.TRANSFER,
                        EnumIntake.ClawPosition.CLOSE,
                        EnumIntake.ClawPosition.OPEN,
                        EnumDeposit.DepositFlipPosition.GRAB,
                        EnumDeposit.FingerPosition.CLOSE,
                        EnumDeposit.DepositFlipPosition.HALF_POSE,
                        500
                ))
                .endTrajectory();

        // Sample deposit
        TrajectoryActionBuilder dropSample1 = pickSample1.endTrajectory().fresh()
                .setTangent(90)
                .lineToYLinearHeading(-55, Math.toRadians(45))
                .endTrajectory();

        // Second sample pickup sequence
        TrajectoryActionBuilder pickSample2 = dropSample1.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-54, -44.5, Math.toRadians(70)), 0)
                .stopAndAdd(robotController.intakeSetPosition(
                        EnumIntake.IntakeExtensionPosition.MAX_EXTENSION,
                        EnumIntake.IntakeSpinPosition.HORIZONTAL,
                        EnumIntake.ClawPosition.OPEN,
                        EnumIntake.IntakeFlip.DOWN,
                        500
                ))
                .stopAndAdd(robotController.fullTransferToDeposit(
                        EnumIntake.IntakeExtensionPosition.TRANSFER,
                        EnumIntake.ClawPosition.CLOSE,
                        EnumIntake.ClawPosition.OPEN,
                        EnumDeposit.DepositFlipPosition.GRAB,
                        EnumDeposit.FingerPosition.CLOSE,
                        EnumDeposit.DepositFlipPosition.HALF_POSE,
                        500
                ))
                .endTrajectory();

        // Sample deposit
        TrajectoryActionBuilder dropSample2 = pickSample2.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(45)), 0)
                .endTrajectory();

        // Third sample pickup sequence
        TrajectoryActionBuilder pickSample3 = dropSample2.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-46, -29, Math.toRadians(-180)), 0)

                .stopAndAdd(robotController.intakeSetPosition(
                        EnumIntake.IntakeExtensionPosition.MAX_EXTENSION,
                        EnumIntake.IntakeSpinPosition.VERTICAL,
                        EnumIntake.ClawPosition.OPEN,
                        EnumIntake.IntakeFlip.DOWN,
                        500
                ))
                .stopAndAdd(robotController.closeIntake(200))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-40,-29,Math.toRadians(180)),0)
                .stopAndAdd(robotController.fullTransferToDeposit(
                        EnumIntake.IntakeExtensionPosition.TRANSFER,
                        EnumIntake.ClawPosition.CLOSE,
                        EnumIntake.ClawPosition.OPEN,
                        EnumDeposit.DepositFlipPosition.GRAB,
                        EnumDeposit.FingerPosition.CLOSE,
                        EnumDeposit.DepositFlipPosition.HALF_POSE,
                        500
                ))
                .endTrajectory();

        // Sample deposit
        TrajectoryActionBuilder dropSample3 = pickSample3.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-55, -56, Math.toRadians(45)), 0)
                .endTrajectory();

        // Combine parallel actions for lift and trajectories
        ParallelAction preloadWithLift = new ParallelAction(
                preload.build(),
                lifts.liftUp(POWER_EXTEND, LIFT_EXTEND_HEIGHT, DELAY)
        );

        ParallelAction pickSample1WithLift = new ParallelAction(
                pickSample1.build(),
                lifts.moveLiftAction(LIFT_RETRACT_HEIGHT-50,lifts) // Retract lift
        );

        ParallelAction dropSample1WithLift = new ParallelAction(
                dropSample1.build(),
                lifts.liftUp(POWER_EXTEND, LIFT_EXTEND_HEIGHT, DELAY)
        );

        ParallelAction pickSample2WithLift = new ParallelAction(
                pickSample2.build(),
                lifts.moveLiftAction(LIFT_RETRACT_HEIGHT-100,lifts)  // Retract lift
        );

        ParallelAction dropSample2WithLift = new ParallelAction(
                dropSample2.build(),
                lifts.liftUp(POWER_EXTEND, LIFT_EXTEND_HEIGHT+2, DELAY)
        );

        ParallelAction pickSample3WithLift = new ParallelAction(
                pickSample3.build(),
                lifts.moveLiftAction(LIFT_RETRACT_HEIGHT+150,lifts)  // Retract lift
        );

        ParallelAction dropSample3WithLift = new ParallelAction(
                dropSample3.build(),
                lifts.liftUp(POWER_EXTEND, LIFT_EXTEND_HEIGHT, DELAY)
        );


        // Execute the actions sequentially
        Actions.runBlocking(robotController.depoSetPosition(EnumDeposit.DepositFlipPosition.GRAB, EnumDeposit.DepositOutPosition.GRAB, EnumDeposit.FingerPosition.CLOSE, EnumDeposit.SpinPosition.SPIN_0,1000));
        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        preloadWithLift,
                        robotController.dumpOpenAndReset(
                                EnumDeposit.DepositFlipPosition.DUMP_OFF,  // Dump position
                                EnumDeposit.FingerPosition.OPEN,           // Open fingers
                                EnumDeposit.DepositFlipPosition.GRAB, // Reset to grab position
                                500),                                        // Delay in milliseconds
                        pickSample1WithLift,
                        dropSample1WithLift,
                                robotController.dumpOpenAndReset(
                                        EnumDeposit.DepositFlipPosition.DUMP_OFF,  // Dump position
                                        EnumDeposit.FingerPosition.OPEN,           // Open fingers
                                        EnumDeposit.DepositFlipPosition.GRAB, // Reset to grab position
                                        500),                                       // Delay in milliseconds
                        pickSample2WithLift,
                        dropSample2WithLift,
                                        robotController.dumpOpenAndReset(
                                                EnumDeposit.DepositFlipPosition.DUMP_OFF,  // Dump position
                                                EnumDeposit.FingerPosition.OPEN,           // Open fingers
                                                EnumDeposit.DepositFlipPosition.GRAB, // Reset to grab position
                                                500),                                      // Delay in milliseconds
                        pickSample3WithLift,
                        dropSample3WithLift,
                        robotController.dumpOpenAndReset(
                                                        EnumDeposit.DepositFlipPosition.DUMP_OFF,  // Dump position
                                                        EnumDeposit.FingerPosition.OPEN,           // Open fingers
                                                        EnumDeposit.DepositFlipPosition.GRAB, // Reset to grab position
                                                        500),                                       // Delay in milliseconds
                        lifts.moveLiftAction(LIFT_RETRACT_HEIGHT,lifts)
                )
        );
    }
}
