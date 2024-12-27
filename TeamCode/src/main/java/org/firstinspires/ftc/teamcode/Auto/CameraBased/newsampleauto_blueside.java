package org.firstinspires.ftc.teamcode.Auto.CameraBased;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ModulesToImport.Conditionals.CombinedDetectionPipeline;
import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.Timer;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.Robot_Controller;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.deposit_subsystem;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.intake_subsystem;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.lifts_subsystem;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Test.OpenCV.CombinedVisionTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
//Use this field layout for X and Y
// -X     BlueR                   BlueL     +X
//|-------------------------------------------|  +Y
//|                                           |
//|       3     1                3     1      | 1
//|          2                      2         | 2 Backdrop
//|                                           | 3
//|                                           |
//|                   0,0                     |
//|                                           |
//|                                           |
//|                                           | 1
//|          2                      2         | 2 Backdrop
//|       1     3                1     3      | 3
//|                                           |
//|-------------------------------------------|  -Y
//         RedL                   RedR
//
//Use this compass for angles to match field above
//              90
//    180                0
//              270
//


@Config
@Autonomous(name="Sample Auto with Camera for Blue", group ="Autonomous")
public class newsampleauto_blueside extends LinearOpMode {


    public class Lifts {
        //private DcMotorEx rightslide;
        //private DcMotorEx leftslide;
        private lifts_subsystem liftss;

        public Lifts(HardwareMap hardwareMap) {
            this.liftss = new lifts_subsystem(hardwareMap, "leftslide", "rightslide");

            telemetry.addData("leftslide", liftss.getLeftSlidePosition());
            telemetry.addData("rightslide", liftss.getRightSlidePosition());
            telemetry.update();

        }

        public class MoveLiftToPosition implements Action {

            private final lifts_subsystem liftSubsystem;
            private final int targetPosition;


            //public MoveLiftToPosition(lifts_subsystem liftSubsystem, int targetPosition) {
            public MoveLiftToPosition(lifts_subsystem liftSubsystem, int targetPosition) {
                this.liftSubsystem = liftSubsystem;
                this.targetPosition = targetPosition; // Passed as a parameter
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                liftSubsystem.slidePID(targetPosition);

                int currentLeft = liftSubsystem.getLeftSlidePosition();
                int currentRight = liftSubsystem.getRightSlidePosition();

                packet.put("Left Slide Position", currentLeft);
                packet.put("Right Slide Position", currentRight);
                telemetry.addData("right", currentRight);
                telemetry.addData("left", currentLeft);
                telemetry.update();

                boolean isAtTarget = // (currentLeft > targetPosition || currentRight > targetPosition)
                        (
                                Math.abs(currentLeft - targetPosition) < 50 ||
                                        Math.abs(currentRight - targetPosition) < 50);

                if (isAtTarget) {
                    liftSubsystem.stopSlides();
                    return false; // Action is complete
                }

                return true; // Action should continue
            }
        }

        public Action moveLiftAction(int targetPosition) {
            return new MoveLiftToPosition(liftss, targetPosition);
        }

        public class Lift implements Action {
            private double power = 0;
            private double ticksToMove = 0;
            private long endTime = 0;

            public Lift(double power, double inches, long secondsToWait) {
                double ticksPerInch = ((4498 + 4345) / 2) / 44.5;

                this.power = power;
                this.ticksToMove = (int) (inches * ticksPerInch);

                liftss.stopandreset();
                liftss.liftrunencoders();


                this.endTime = System.currentTimeMillis() + secondsToWait * 1000;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (System.currentTimeMillis() < endTime) {
                    return true;
                }

                if (Math.abs(liftss.getLeftSlidePosition()) < ticksToMove) { //&& Math.abs(getRightSlidePosition()) < ticksToMove) {
                    liftss.runslides(power);
                    return true;
                }
                liftss.stopSlides();
                return false;
            }
        }

        public Action liftUp(double power, double inches, long secondsToWait) {
            return new Lift(power, inches, secondsToWait);
        }

        public Action liftDown(double power, double inches, long secondsToWait) {
            return new Lift(-power, inches, secondsToWait);
        }
    }



        /*

 /*
*/

        public class Robot_controller {

            private Robot_Controller attachments;

            public Robot_controller(intake_subsystem intake_sub, deposit_subsystem depo_sub) {
                this.attachments = new Robot_Controller(intake_sub, depo_sub);
            }

            public class intake_ext implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    try {
                        attachments.ext_max();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    return false;
                }
            }

            public Action intake_ext_max() {
                return new intake_ext();
            }

            public class intake_ext_3rd implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    try {
                        attachments.ext_max_3rd();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    return false;
                }
            }

            public Action intake_ext_max_3rd() {
                return new intake_ext_3rd();
            }

            public class transferpose implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    try {
                        attachments.transfer();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    return false;
                }
            }

            public Action transfer() {
                return new transferpose();
            }

            public class intake_open implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    attachments.int_open();
                    return false;
                }
            }

            public Action intake_open() {
                return new intake_open();
            }

            public class intake_close implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    attachments.int_close();
                    return false;
                }
            }

            public Action intake_close() {
                return new intake_close();
            }

            public class int_spin_vert implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    try {
                        attachments.int_vert_spin();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    return false;
                }
            }

            public Action int_vert() {
                return new int_spin_vert();
            }

            public class int_spin_hor implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    attachments.int_horizontal_spin();
                    return false;
                }
            }

            public Action int_hor() {
                return new int_spin_hor();
            }

            public class preint implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    try {
                        attachments.preintake();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    return false;
                }
            }

            public Action preint() {
                return new finger_close_pose();
            }

            public class droptake implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    attachments.dropintake();
                    return false;
                }
            }

            public Action droptake() {
                return new finger_close_pose();
            }

            //claw/deposit time

            public class depo_grab_pose implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    try {
                        attachments.depotransfer();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    return false;
                }
            }

            public Action depo_grab() {
                return new depo_grab_pose();
            }

            public class depo_half_pose implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    try {
                        attachments.drophalfwaypose();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    return false;
                }
            }

            public Action depo_half() {
                return new depo_half_pose();
            }

            public class depo_dump_pose implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    try {
                        attachments.droppose();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    return false;
                }
            }

            public Action depo_dump() {
                return new depo_dump_pose();
            }

            public class depo_down_pose implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    try {
                        attachments.goingdown();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    return false;
                }
            }

            public Action depo_down() {
                return new depo_down_pose();
            }

            public class depo_preload_pose implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    try {
                        attachments.depopreload();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    return false;
                }
            }

            public Action depo_preload() {
                return new depo_preload_pose();
            }

            public class fing_open_pose implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    attachments.fingeropen();
                    sleep(200);
                    return false;
                }
            }

            public Action fing_open() {
                return new fing_open_pose();
            }

            public class finger_close_pose implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    attachments.fingerclose();
                    return false;
                }
            }

            public Action finger_close() {
                return new finger_close_pose();
            }

        }

            public class StopStreaming implements Action {
                private final OpenCvCamera webcam;
                private boolean isComplete = false;

                public StopStreaming(OpenCvCamera webcam) {
                    this.webcam = webcam;
                }

                @Override
                public boolean run(TelemetryPacket packet) {
                    if (!isComplete) {
                        webcam.stopStreaming();
                        isComplete = true; // Mark the action as complete after stopping the stream
                    }
                    return false; // Action completes immediately
                }
            }

            public class StartStreaming implements Action {
                private final OpenCvCamera webcam;
                private final int width;
                private final int height;
                private final OpenCvCameraRotation rotation;
                private boolean isComplete = false;

                public StartStreaming(OpenCvCamera webcam, int width, int height, OpenCvCameraRotation rotation) {
                    this.webcam = webcam;
                    this.width = width;
                    this.height = height;
                    this.rotation = rotation;
                }

                @Override
                public boolean run(TelemetryPacket packet) {
                    if (!isComplete) {
                        webcam.startStreaming(width, height, rotation);
                        isComplete = true; // Mark the action as complete after starting the stream
                    }
                    return false; // Action completes immediately
                }
            }
        //camera action
    public class PipelineAction implements Action {
        private final CombinedDetectionPipeline pipeline;
        private final Runnable onDetectionComplete;
        private boolean isComplete = false;

        public PipelineAction(CombinedDetectionPipeline pipeline, Runnable onDetectionComplete) {
            this.pipeline = pipeline;
            this.onDetectionComplete = onDetectionComplete;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            // Continuously fetch detection results
            String color = pipeline.getDominantColor();
            String orientation = pipeline.getOrientation();

            // Update telemetry with current detection data
            packet.put("Detected Color", color);
            packet.put("Detected Orientation", orientation);

            // Add logic for stopping or completing the action
            if (!"Unknown".equals(color) && !"Unknown".equals(orientation)) {
                // Trigger the callback to handle detection
                onDetectionComplete.run();
                isComplete = true;
            }

            return !isComplete; // Continue running until detection is complete
        }
    }



        public MecanumDrive drivetrain;
        public ColorSensor colorSensor;
        //init the poses
        public Pose2d initialpose;


        //color sensor Values
        public double redValue;
        public double greenValue;
        public double blueValue;
        public double alphaValue;
        public double targetValue = 1000;

        public  OpenCvCamera webcam;
        public CombinedDetectionPipeline pipeline;

        public long starttime = System.currentTimeMillis();



    @Override
        public void runOpMode() {

            //setting up camera
            initCamera(pipeline); // Pass the pipeline to your camera initialization method


            // define the poses
            initialpose = new Pose2d(-38.5, -64, Math.toRadians(0));
            // declaring the drivetrain
            MecanumDrive drivetrain = new MecanumDrive(hardwareMap, initialpose);

            //calling attachments
            Lifts lifts = new Lifts(hardwareMap);
            //needed to intialize the robot controller
            intake_subsystem intake_sub = new intake_subsystem(hardwareMap, "left_intake_flip", "right_intake_flip", "left_extension", "right_extension", "clawtakespin", "clawtake");
            deposit_subsystem depo_sub = new deposit_subsystem(hardwareMap, "left_outtake_flip", "right_outtake_flip", "outtake_ext", "spin", "finger");
            Robot_controller servo_attachments = new Robot_controller(intake_sub, depo_sub);
            // start streaming Action
        Action startStreaming = new StartStreaming(webcam, 320, 240, OpenCvCameraRotation.UPRIGHT);
        Action stopStreaming = new StopStreaming(webcam);


        // Define the action for camera and its happening based on color
        Action SampleDetection = new PipelineAction(pipeline, () -> {
            // Code to execute when detection is complete
            String detectedColor = pipeline.getDominantColor();
            String detectedOrientation = pipeline.getOrientation();
            telemetry.addData("Detected Color", detectedColor);
            telemetry.addData("Detected Orientation", detectedOrientation);
            telemetry.update();
            if("Veritcal".equals(detectedOrientation)) {
                if ("Blue".equals(detectedColor) || "Yellow".equals(detectedColor)) {
                    servo_attachments.int_vert();
                    sleep(100);
                    servo_attachments.transfer();
                } else if (System.currentTimeMillis() - starttime < 25000) {
                    while(!("Blue".equals(detectedColor) || "Yellow".equals(detectedColor))){
                        strafeLeft(drivetrain);
                        detectedColor = pipeline.getDominantColor();
                    }
                }
            } else if ("Horizontal".equals(detectedOrientation)){
                if ("Blue".equals(detectedColor) || "Yellow".equals(detectedColor)) {
                    servo_attachments.int_hor();
                    sleep(100);
                    servo_attachments.transfer();
                } else if (System.currentTimeMillis() - starttime < 25000) {
                    while(!("Blue".equals(detectedColor) || "Yellow".equals(detectedColor))){
                        strafeLeft(drivetrain);
                        detectedColor = pipeline.getDominantColor();
                    }
                }
            } else if ("Unknown".equals(detectedOrientation) && (System.currentTimeMillis() - starttime < 25000)){
                while(!("Blue".equals(detectedColor) || "Yellow".equals(detectedColor))){
                    strafeLeft(drivetrain);
                    detectedColor = pipeline.getDominantColor();
                }
            }
        });


        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
            getColor();
            colorTelementry();


            // defining each trajectory
            //goes to drop off preload
            TrajectoryActionBuilder preload = drivetrain.actionBuilder(initialpose)
                    .setTangent(Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(-56,-55, Math.toRadians(45.6557)),0)
                    .endTrajectory();
            TrajectoryActionBuilder picksample1 = preload.endTrajectory().fresh()
                    .setTangent(90)
                    .splineToLinearHeading(new Pose2d(-55, -53, Math.toRadians(90)), 90)
                    .endTrajectory();
            TrajectoryActionBuilder dropsample1 = picksample1.endTrajectory().fresh()
                    .setTangent(90)
                    .turnTo(Math.toRadians(45))
                    .endTrajectory();
            TrajectoryActionBuilder picksample2 = dropsample1.endTrajectory().fresh()
                    //need to find the exact thing maybe forward very little
                    .endTrajectory();
            TrajectoryActionBuilder dropsample2 = picksample2.endTrajectory().fresh()
                    //shouldn't need to move will ave to check
                    .endTrajectory();
            TrajectoryActionBuilder picksample3parallel = dropsample2.endTrajectory().fresh()
                    .setTangent(0)
                    .turnTo(Math.toRadians(125))
                    .endTrajectory();
            TrajectoryActionBuilder dropsample3 = picksample3parallel.endTrajectory().fresh()
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-55, -53, Math.toRadians(90)), 0)
                    .endTrajectory();
            TrajectoryActionBuilder middlesample_park = dropsample3.endTrajectory().fresh()
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-29, -10, Math.toRadians(0)), 0)
                    .endTrajectory();
            // goes back a little

            Action traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8;
            traj1 = preload.build();
            traj2 = picksample1.build();
            traj3 = dropsample1.build();
            traj4 = picksample2.build();
            traj5 = dropsample2.build();
            traj6 = picksample3parallel.build();
            traj7 = dropsample3.build();
            traj8 = middlesample_park.build();


            // sets of parallel actions
            ParallelAction p1 = new ParallelAction(
                    traj1,
                    servo_attachments.depo_preload(),
                    lifts.liftUp(1.0, 36, 0)
            );
            ParallelAction p2 = new ParallelAction(
                    traj2,
                    lifts.moveLiftAction(0)
            );

            ParallelAction p3 = new ParallelAction(
                    traj3,
                    servo_attachments.depo_half(),
                    lifts.liftUp(1.0, 36, 0)
            );

            ParallelAction p4 = new ParallelAction(
                    traj4,
                    lifts.moveLiftAction(0),
                    servo_attachments.depo_grab()
            );
            ParallelAction p5 = new ParallelAction(
                    traj5,
                    servo_attachments.depo_half(),
                    lifts.liftUp(1.0, 36, 0)
            );

            ParallelAction p6 = new ParallelAction(
                    traj6,
                    lifts.moveLiftAction(0),
                    servo_attachments.depo_grab()
            );
            ParallelAction p7 = new ParallelAction(
                    traj7,
                    servo_attachments.depo_half(),
                    lifts.liftUp(1.0, 36, 0)
            );
            ParallelAction p8 = new ParallelAction(
                    lifts.moveLiftAction(0),
                    traj8
            );


// actual segment that runs, parallel actions and sequential order
            //Actions.runBlocking(servo_attachments.transfer());
            Actions.runBlocking(servo_attachments.depo_grab());
            telemetry.addLine("Mykel like boys");
            telemetry.addLine("Start on me start on three wooo");
            Actions.runBlocking(servo_attachments.finger_close());

            //Actions.runBlocking(lifts.initlift());
            waitForStart();
            Actions.runBlocking(
                    new SequentialAction(
                            p1,
                            servo_attachments.depo_dump(),
                            servo_attachments.fing_open(),


                            p2,
                            servo_attachments.intake_ext_max(),
                            servo_attachments.transfer(),
                            servo_attachments.depo_half(),


                            p3,
                            servo_attachments.depo_dump(),
                            servo_attachments.fing_open(),


                            p4,
                            servo_attachments.intake_ext_max(),
                            servo_attachments.transfer(),
                            servo_attachments.depo_half(),


                            p5,
                            servo_attachments.depo_dump(),
                            servo_attachments.fing_open(),


                            p6,
                            servo_attachments.intake_ext_max(),
                            servo_attachments.transfer(),
                            servo_attachments.depo_half(),


                            p7,
                            servo_attachments.depo_dump(),
                            servo_attachments.fing_open(),
                            servo_attachments.depo_grab(),

                            p8//,

                            // how to call camera during the auto
                            //startStreaming,
                            //SampleDetection,
                            //stopStreaming,
                            //




                    )
            );
            // for run action building out everytging under a squential action and then inside of tha tput different parallel commands

        }

        public void getColor() {
            redValue = colorSensor.red();
            greenValue = colorSensor.green();
            blueValue = colorSensor.blue();
            alphaValue = colorSensor.alpha();
        }

        public void colorTelementry() {
            telemetry.addData("redValue", "%.3f", redValue);
            telemetry.addData("greenValue", "%.3f", greenValue);
            telemetry.addData("blueValue", "%.3f", blueValue);
            telemetry.addData("alphaValue", "%.3f", alphaValue);
            telemetry.update();

        }
    private void initCamera(CombinedDetectionPipeline pipeline) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        this.pipeline = new CombinedDetectionPipeline();
        webcam.setPipeline(this.pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });
    }

    public void strafeLeft(MecanumDrive drivetrain){
        drivetrain.leftFront.setPower(0.2);
        drivetrain.leftBack.setPower(-0.2);
        drivetrain.rightFront.setPower(-0.2);
        drivetrain.rightBack.setPower(0.2);
    }

    public void strafeRight(MecanumDrive drivetrain){
        drivetrain.leftFront.setPower(-0.2);
        drivetrain.leftBack.setPower(0.2);
        drivetrain.rightFront.setPower(0.2);
        drivetrain.rightBack.setPower(-0.2);
    }

}

