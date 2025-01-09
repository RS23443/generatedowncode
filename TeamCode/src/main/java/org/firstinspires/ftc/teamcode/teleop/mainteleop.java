package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ModulesToImport.Conditionals.EnhancedPipeline;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.hang_subsystem;
import org.firstinspires.ftc.teamcode.Test.OpenCV.SmallerReferenceTest;
import org.firstinspires.ftc.teamcode.Test.throughBoreDifferential;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp(name="main", group="LinearOpMode")
public class mainteleop extends LinearOpMode {
    // wifi name: 23443-RC, password:RoboSupport23443

    public DcMotorEx frmotor; // control hub 2
    public DcMotorEx brmotor; // control hub 3
    public DcMotorEx flmotor; // control hub 0
    public DcMotorEx blmotor; //control hub 1
    private DcMotorEx leftslide; //exapnsion hub 0
    private DcMotorEx rightslide; //expansion hub 1

    public ColorSensor clawColorSensor;
    public double clawredValue, clawgreenValue, clawalphaValue, clawblueValue;
    private double targetValue = 1000;

    public double x;
    public double rx;
    public double y;
    public double ry;
    public double denominator;
    public double speed = 1.0;
    //public static int servorpm = 230; // Servo RPM for calculations
    //public static double power = 0.4; // Power level to apply to servos
    //public static long targetRotationTime = 1000 * 60 / servorpm; // Time for one full rotation (in ms)
    //public static long targetSpinTime = 750 * 60 / servorpm;

    //private long starttime = 0;
    //private boolean isRunning = false; // Tracks whether servos are running
    private int action = 0; // Tracks the current action: 0 = none, 1 = A, 2 = B, 3 = X, 4 = Y
    public boolean isBlockedGrabbed = false;
    //public TouchSensor transferTouch;
    //public TouchSensor intakeTouch;
    //public boolean isTransferTouchPressed = false;
    //public boolean isIntakeTouchPressed = false;
    //private double transferTouchValue;
    //private double intakeTouchValue;
    //private boolean stopdifferential;
    //public throughBoreDifferential tbore;
    public boolean isSnapped = true;
    public boolean isReadyToTransfer = false;
    OpenCvCamera webcam;
    EnhancedPipeline pipeline;


    @Override
    public void runOpMode() throws InterruptedException {
        //controller = new PIDController(p, i, d);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //controller1 = new PIDController(p1,i1,d1);

        //Servos
        leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
        Servo left_intake_extension = hardwareMap.get(Servo.class, "left_intake_ext");
        Servo left_intake_flip = hardwareMap.get(Servo.class, "left_intake_flip");
        Servo left_intake_diffy = hardwareMap.get(Servo.class, "left_intake_diffy");
        Servo intake_claw = hardwareMap.get(Servo.class, "intake_claw");

        // Expansion Hub, order 3 to 5/ top-down
        Servo right_intake_diffy = hardwareMap.get(Servo.class, "right_intake_diffy");
        Servo right_intake_flip = hardwareMap.get(Servo.class, "right_intake_flip");
        Servo right_intake_extension = hardwareMap.get(Servo.class, "right_intake_ext");

        // outtake servos
        // Control Hub, order 4 to 5/ top-down
        Servo left_outtake_flip = hardwareMap.get(Servo.class, "left_outtake_flip");
        Servo left_outtake_diffy = hardwareMap.get(Servo.class, "left_outtake_diffy");
        // Expansion Hub, order 0 to 2/ top-down
        Servo outttake_claw = hardwareMap.get(Servo.class, "outtake_claw");
        Servo right_outtake_diffy = hardwareMap.get(Servo.class, "right_outtake_diffy");
        Servo right_outake_flip = hardwareMap.get(Servo.class, "right_outtake_flip");

        //Motors
        frmotor = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        flmotor = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        brmotor = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        blmotor = hardwareMap.get(DcMotorEx.class, "left_back_drive");


        // reverse any Motors
        rightslide.setDirection(DcMotorEx.Direction.REVERSE);
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hang_subsystem hooks = new hang_subsystem(hardwareMap,"right_hook", "left_hook");

        //sensors
        clawColorSensor = hardwareMap.get(ColorSensor.class, "clawColorSensor");
        //transferTouch = hardwareMap.get(TouchSensor.class, "transfer_touch");
        //intakeTouch = hardwareMap.get(TouchSensor.class,"intake_touch");
        //tbore = new throughBoreDifferential(hardwareMap,"left_hook", "right_hook");
        initCamera();

        //for encoders reverse left
        waitForStart();
        while (opModeIsActive()) {
            // setting up of sensors for continuous run
            getColor();
            colorTelementry();
            //getTransferTouch();
            //getIntakeTouch();
            //touchSensorTelem();
            //stopdifferential = tbore.shouldStop();


            String detectedColor = pipeline.getDominantColor();
            String detectedOrientation = pipeline.getOrientation();

            telemetry.addData("Detected Color", detectedColor);
            telemetry.addData("Detected Orientation", detectedOrientation);
            telemetry.update();


            // assigning actions
            if (gamepad2.left_bumper) {
                //max
                outttake_claw.setPosition(0.58);
                sleep(100);
                intake_claw.setPosition(0.15);
                sleep(100);
                left_outtake_flip.setPosition(0.7); // 0.7 value for the sample
                right_outake_flip.setPosition(0.3); // 0.3 value for the sample
                sleep(100);
                //teleop diffy pose btw
                right_outtake_diffy.setPosition(0.75);
                left_outtake_diffy.setPosition(0.25);
            }

            if (gamepad2.right_bumper) {
                outttake_claw.setPosition(0.35);
                right_outtake_diffy.setPosition(0.75);
                left_outtake_diffy.setPosition(0.75);
                left_outtake_flip.setPosition(0.0);
                right_outake_flip.setPosition(1.0);
                isBlockedGrabbed = false;
            }

            if (gamepad2.y && isBlockedGrabbed && (clawalphaValue > 900)) {
                outttake_claw.setPosition(0.35);
            }

            if(gamepad2.a){
                left_outtake_diffy.setPosition(0.0);
                right_outtake_diffy.setPosition(1.0);
            }

            if (gamepad1.a /*&& !isRunning*/) {
                //flip down
                left_intake_flip.setPosition(0.39);
                right_intake_flip.setPosition(0.61);
                //transfer
                intake_claw.setPosition(0.35);//open is at 0.15
                sleep(200);
                left_intake_diffy.setPosition(0.1); // towards 1
                right_intake_diffy.setPosition(0.9); // towards 0
                //startAction(2, left_intake_diffy, right_intake_diffy, -1, 1);//flip the diffy
                //then flip the intake in
                sleep(50);
                left_intake_flip.setPosition(0.55);//0.65
                right_intake_flip.setPosition(0.45); //0.35
                isReadyToTransfer = true;
                //sleep(100);
                // Bring back the extension
                //left_intake_extension.setPosition(1.0); // most likely
                //right_intake_extension.setPosition(0.0);
            }

            if(gamepad1.right_bumper || isReadyToTransfer){
                left_intake_flip.setPosition(0.65);//0.65
                right_intake_flip.setPosition(0.35); //0.35
                sleep(100);
                // Bring back the extension
                left_intake_extension.setPosition(1.0); // most likely
                right_intake_extension.setPosition(0.0);
                isReadyToTransfer = false;

            }

            if (gamepad1.b /*&& !isRunning*/) {
                //ready to grab
                //extend
                left_intake_extension.setPosition(0.8);
                right_intake_extension.setPosition(0.2);
                isSnapped = false;
                //sleep(100);
                //intake_claw.setPosition(0.15);//open is at 0.15
                //startAction(3, left_intake_diffy, right_intake_diffy, power, -power);
                //left_intake_flip.setPosition(0.45);
                //right_intake_flip.setPosition(0.55);
                //sleep(100);
            }

            if (gamepad1.x /*&& !isRunning*/) {
                //startAction(1, left_intake_diffy, right_intake_diffy, power, power);
            }
            if (gamepad1.y /*&& !isRunning*/) {

                //startAction(4, left_intake_diffy, right_intake_diffy, -power, -power);
            }

           /* if("Horizontal".equals(detectedOrientation)){
                startAction(1, left_intake_diffy, right_intake_diffy, power, power);
            }*/
            if("Yellow".equals(detectedColor) || "Blue".equals(detectedColor)){
                if("Vertical".equals(detectedOrientation) && !isSnapped){
                    //bring the claw down
                    //vertical block
                    /*
                    |         |
                    |         |
                    |         |
                    |         |
                    |         |
                    ------------
                     */
                    intake_claw.setPosition(0.15);//open is at 0.15
                    sleep(50);
                    left_intake_diffy.setPosition(0.6); // towards 1
                    right_intake_diffy.setPosition(0.4); // towards 0
                    //startAction(3, left_intake_diffy, right_intake_diffy, power, -power);
                    left_intake_flip.setPosition(0.5);
                    right_intake_flip.setPosition(0.5);
                    //sleep(1000);
                    //intake_claw.setPosition(0.35);//open is at 0.15
                    //sleep(200);
                    //startAction(2, left_intake_diffy, right_intake_diffy, -power - 0.1, power + 0.1);//flip the diffy
                    //then flip the intake in
                    //flip down
                    //left_intake_flip.setPosition(0.44);
                    //right_intake_flip.setPosition(0.56);
                    //sleep(1000);
                    isSnapped = true;
                }
                if("Horizontal".equals(detectedOrientation) && !isSnapped){
                    //horizontal block
                     /*
                     --------------
                     |            |
                     |            |
                     --------------
                     */
                    //spin
                    left_intake_diffy.setPosition(0.2); // 1.0
                    right_intake_diffy.setPosition(0.0); //1.0
                    sleep(50);
                    //startAction(1, left_intake_diffy, right_intake_diffy, power, power);
                    // drop
                    intake_claw.setPosition(0.15);//open is at 0.15
                    //startAction(3, left_intake_diffy, right_intake_diffy, power, -power);
                    left_intake_flip.setPosition(0.5);
                    right_intake_flip.setPosition(0.5);
                    sleep(100);
                    // grab
                   // intake_claw.setPosition(0.35);//open is at 0.15
                   // sleep(200);
                    // spin back
                    //startAction(4, left_intake_diffy, right_intake_diffy, -power - 0.1, power + 0.1);//flip
                    //flip down
                    telemetry.addLine("horizontal action being preformed");
                    isSnapped = true;
                }

            }


            /*
            if ((action == 2) && isRunning && isTransferTouchPressed) {
                left_intake_diffy.setPower(0.0);
                right_intake_diffy.setPower(0.0);
                isRunning = false; // Reset the running state
                action = 0; // Clear the current action
            } else if ((action == 3) && isRunning && isIntakeTouchPressed) {
                left_intake_diffy.setPower(0.0);
                right_intake_diffy.setPower(0.0);
                isRunning = false; // Reset the running state
                action = 0; // Clear the current action
            } else if (isRunning) {
                if (System.currentTimeMillis() - starttime > 100) {
                    left_intake_diffy.setPower(0.0);
                    right_intake_diffy.setPower(0.0);
                    isRunning = false; // Reset the running state
                    action = 0; // Clear the current action
                }
            }*/

            if (clawalphaValue > 900 && !isBlockedGrabbed) {
                outttake_claw.setPosition(0.58);
                sleep(50);
                intake_claw.setPosition(0.15);
                sleep(50);
                left_intake_extension.setPosition(0.96);
                right_intake_extension.setPosition(0.04);
                sleep(50);
                left_outtake_flip.setPosition(0.7); // 0.7
                right_outake_flip.setPosition(0.3); // 0.3
                sleep(100);
                //teleop diffy pose btw
                right_outtake_diffy.setPosition(0.75);
                left_outtake_diffy.setPosition(0.25);
                isBlockedGrabbed = true;
            }
            /*
            // will needed to put under another coditional to ensure it only happens once
            tbore.resetCumulativeDifferences();
            startAction(3, left_intake_diffy, right_intake_diffy, power, -power);
            while (!tbore.shouldStop()) {
                tbore.updateCumulativeDifferences();
                Thread.sleep(10); // Avoid busy-waiting and allow other threads to run
            }
            // Stop the servos after exiting the loop
            left_intake_diffy.setPower(0.0);
            right_intake_diffy.setPower(0.0);
            action = 0;
            isRunning = false;

             */



            rx = gamepad1.right_stick_x;
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            ry = -gamepad1.right_stick_y;

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            flmotor.setPower((y + rx + x) / denominator * speed);
            blmotor.setPower((y - rx + x) / denominator * speed);
            frmotor.setPower((y - rx - x) / denominator * speed);
            brmotor.setPower((y + rx - x) / denominator * speed);


            rightslide.setPower(gamepad2.left_stick_y * speed);
            leftslide.setPower(gamepad2.left_stick_y * speed);
            lift_telemtry();

            hooks.joysticks(gamepad2,1.0);

        }
    }

    public void getColor() {
        clawredValue = clawColorSensor.red();
        clawgreenValue = clawColorSensor.green();
        clawblueValue = clawColorSensor.blue();
        clawalphaValue = clawColorSensor.alpha();
    }

    public void colorTelementry() {
        telemetry.addData("redValue", "%.3f", clawredValue);
        telemetry.addData("greenValue", "%.3f", clawgreenValue);
        telemetry.addData("blueValue", "%.3f", clawblueValue);
        telemetry.addData("alphaValue", "%.3f", clawalphaValue);
        telemetry.update();

    }

    public void lift_telemtry() {
        telemetry.addData("LL", leftslide.getCurrentPosition());
        telemetry.addData("RL", rightslide.getCurrentPosition());
        telemetry.update();
    }

    /*private void startAction(int actionId, CRServo leftServo, CRServo rightServo, double leftPower, double rightPower) {
        // Start a new action
        starttime = System.currentTimeMillis();
        leftServo.setPower(leftPower);
        rightServo.setPower(rightPower);
        isRunning = true;
        action = actionId;
    }

    private void getTransferTouch(){
        isTransferTouchPressed = transferTouch.isPressed();
        transferTouchValue = transferTouch.getValue();
    }

    private void getIntakeTouch(){
        isIntakeTouchPressed = intakeTouch.isPressed();
        intakeTouchValue = intakeTouch.getValue();
    }

    private void touchSensorTelem(){
        telemetry.addData("Transfer Complete", isTransferTouchPressed);
        telemetry.addData("Transfer Touch Sensor Value","%.2f", transferTouchValue);
        telemetry.addData("ready to grab", isIntakeTouchPressed);
        telemetry.addData("Intake Touch Sensor Value", "%.2f", intakeTouchValue);
        telemetry.update();
    }*/
    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new EnhancedPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });
    }
}