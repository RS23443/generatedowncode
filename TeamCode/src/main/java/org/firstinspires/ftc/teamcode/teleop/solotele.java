package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.deposit_subsystem;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.hang_subsystem;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.intake_subsystem;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.lifts_subsystem;

@Config
@TeleOp(name = "solo teleop", group = "LinearOpMode")
public class solotele extends LinearOpMode {

    // Subsystems
    public MecanumDrive drivetrain;
   public lifts_subsystem lifts;
    public hang_subsystem hang;
    public intake_subsystem intake;
    public deposit_subsystem deposit;

    // Sensors
    public ColorSensor clawColorSensor;

    // States
    public PIDController controller;
    public static double p = 0.014, i = 0.0025, d = 0.000;
    public static double f = 0.075;
    public PIDController controller1;

    public static double p1 = 0.014 , i1 = 0.0025, d1 = 0.000;
    public static double f1 = 0.075;

    public static int target = 500;
    private final double ticks_in_degree = 145.1 / 180.0;
    public Pose2d startpose = new Pose2d(0,29,0);
    public boolean isBlockedGrabbed = false;
    public boolean isSnapped = true;
    public boolean isReadyToTransfer = false;
    public boolean isIntakeMode = true;
    public boolean isOuttakeMode = false;
    public boolean isDepoInGrabPose = true;
    public boolean inHangMode = false;
    // Controls
    public double x, y, rx, denominator;
    public double speed = 1.0;
    private double clawAlphaValue;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        intake = new intake_subsystem(hardwareMap,
                "left_intake_flip", "right_intake_flip",
                "left_intake_ext", "right_intake_ext",
                "right_intake_diffy", "left_intake_diffy",
                "intake_claw");

        deposit = new deposit_subsystem(intake, hardwareMap,
                "left_outtake_flip", "right_outtake_flip",
                "left_outtake_diffy", "right_outtake_diffy",
                "outtake_claw");


       lifts = new lifts_subsystem(hardwareMap,"leftslide", "rightslide");

        hang = new hang_subsystem(hardwareMap, "right_hook", "left_hook");
        drivetrain = new MecanumDrive(hardwareMap,startpose); // Initialize drivetrain

        // Initialize sensors
        clawColorSensor = hardwareMap.get(ColorSensor.class, "clawColorSensor");

        // Camera initialization
        telemetry.addLine("Initializing camera...");
        telemetry.update();

        intake.initCamera(hardwareMap, telemetry);

        // Wait for camera initialization
        while (!intake.isCameraInitialized()) {
            telemetry.addLine("Waiting for camera to initialize...");
            telemetry.update();
            sleep(50); // Wait in small increments to reduce delays
        }

        telemetry.addLine("Camera ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update sensors
            updateClawColorSensor();

            // Display telemetry
            telemetry.addData("Detected Color", intake.getPipeline().getDominantColor());
            telemetry.addData("Detected Orientation", intake.getPipeline().getOrientation());
            telemetry.update();

            // Assign actions
            if (gamepad1.dpad_down) {
                isIntakeMode = true;
                isOuttakeMode = false;
            }

            if (gamepad1.dpad_up) {
                isIntakeMode = false;
                isOuttakeMode = true;
            }

            if(gamepad1.dpad_right){
                inHangMode = true;
            }

            // Handle intake or outtake based on mode
            if (isIntakeMode) {
                handleIntakeControls();
            }

            if (isOuttakeMode) {
                handleOuttakeControls();
            }

            // Deposit block when ready
            if (clawAlphaValue > 900 && isReadyToTransfer && isDepoInGrabPose) {
                deposit.dropose();
                isBlockedGrabbed = true;
                isReadyToTransfer = false;
                isDepoInGrabPose = false;
            }

            if (!isDepoInGrabPose && !isBlockedGrabbed) {
                deposit.grabpose();
                isDepoInGrabPose = true;
            }
            lifts.joystick(gamepad1,1.0);

            hang.upTriggerBased(gamepad1,1.0);
            hang.DownTriggerBased(gamepad1,1.0);

            // Drive controls
            handleDriveControls();


        }
    }

    private void handleIntakeControls() throws InterruptedException {
        if (gamepad1.a) {
            intake.dropdownintake();
        }

        if(gamepad1.right_bumper){
            intake.transferpose();
            isReadyToTransfer = true;
        }

        if (gamepad1.b) {
            intake.prefireintake();
            isSnapped = false;
        }

        if (gamepad1.x) {
            intake.intakespinhorizontal();
        }

        if (gamepad1.y) {
            intake.intakespinvertical();
        }

        String detectedColor = intake.getPipeline().getDominantColor();
        String detectedOrientation = intake.getPipeline().getOrientation();

        if ("Yellow".equals(detectedColor) || "Blue".equals(detectedColor)) {
            if ("Vertical".equals(detectedOrientation) && !isSnapped) {
                intake.intakeopen();
                intake.intakespinvertical();
                isSnapped = true;
            }

            if ("Horizontal".equals(detectedOrientation) && !isSnapped) {
                intake.intakeopen();
                intake.intakespinhorizontal();
                telemetry.addLine("Horizontal action performed");
                isSnapped = true;
            }
        }
    }

    private void handleOuttakeControls() {
        if (gamepad1.b) {
            try {
                deposit.grabpose();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            isBlockedGrabbed = false;
            isDepoInGrabPose = true;
        }

        if (gamepad1.y && isBlockedGrabbed && clawAlphaValue > 900) {
            deposit.open();
            isBlockedGrabbed = false;
        }

        if (gamepad1.a) {
            deposit.spintospec();
        }

        if(gamepad1.x){
            deposit.close();
            isBlockedGrabbed = true;
        }
    }

    private void handleDriveControls() {
        //rx = gamepad1.right_stick_x;
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;

        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        drivetrain.leftFront.setPower((y /* +rx*/ + x) / denominator * speed);
        drivetrain.leftBack.setPower((y  /*-rx*/ + x) / denominator * speed);
        drivetrain.rightFront.setPower((y /*- rx*/ - x) / denominator * speed);
        drivetrain.rightBack.setPower((y /*+ rx*/ - x) / denominator * speed);
    }

    private void updateClawColorSensor() {
        clawAlphaValue = clawColorSensor.alpha();
        telemetry.addData("Claw Alpha", clawAlphaValue);
    }
}

