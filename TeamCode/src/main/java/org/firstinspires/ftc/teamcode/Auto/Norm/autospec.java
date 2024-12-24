package org.firstinspires.ftc.teamcode.Auto.Norm;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.lifts_subsystem;
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

@Disabled
@Config
@Autonomous(name="4 spec", group ="Autonomous")
public class autospec extends LinearOpMode {
    public class Lifts {
        private lifts_subsystem lifts_subsystem;
        private DcMotor rightslide;
        private DcMotor leftslide;

        public Lifts(HardwareMap hardwareMap) {
            rightslide = hardwareMap.get(DcMotor.class, "rightslide");
            leftslide = hardwareMap.get(DcMotor.class, "leftslide");

            leftslide.setDirection((DcMotor.Direction.REVERSE));

            leftslide.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
            rightslide.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
            leftslide.setTargetPosition(0);
            rightslide.setTargetPosition(0);

           leftslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           rightslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

           rightslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           leftslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
        public int rightgetposition(){
            int pos2;
            pos2 = rightslide.getCurrentPosition();
            return pos2;
        }

        public int leftgetposition(){
            int pos3;
            pos3 = leftslide.getCurrentPosition();
            return pos3;
        }
        public void slideTelementry() {
            telemetry.addData("leftslidepose",leftgetposition());
            telemetry.addData("rightslidepose",rightgetposition());

        }
        public class Initifts implements Action {

            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    rightslide.setPower(1.0);
                    leftslide.setPower(1.0);
                    initialized = true;
                }

                // checks lift's current position
                double pos = rightslide.getCurrentPosition();
                double pos1 = leftslide.getCurrentPosition();
                packet.put("rightSlidePos", pos);
                packet.put("leftSlidePos", pos1);
                telemetry.addData("rightslidepose",pos1);
                telemetry.addData("leftslide",pos);
                if (pos < 500.00 || pos1 < 500.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    rightslide.setPower(0);
                    leftslide.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 2000 encoder ticks, then powers it off
            }
        }
        public Action initlift(){
            return new Initifts();
        }

        public class SpecLiftUp implements Action {

            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    rightslide.setPower(1.0);
                    leftslide.setPower(1.0);
                    initialized = true;
                }

                // checks lift's current position
                double pos = rightslide.getCurrentPosition();
                double pos1 = leftslide.getCurrentPosition();
                packet.put("rightSlidePos", pos);
                packet.put("leftSlidePos", pos1);
                telemetry.addData("rightslidepose",pos1);
                telemetry.addData("leftslide",pos);
                if (pos < 2000.0 || pos1 < 2000.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    rightslide.setPower(0);
                    leftslide.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 2000 encoder ticks, then powers it off
            }
        }

        public Action SpecLiftUp() {
            return new Lifts.SpecLiftUp();
        }

        public class sampleLiftUp implements Action {
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    rightslide.setPower(0.8);
                    leftslide.setPower(0.8);
                    initialized = true;
                }

                // checks lift's current position
                double pos = rightslide.getCurrentPosition();
                double pos1 = leftslide.getCurrentPosition();
                packet.put("rightSlidePos", pos);
                packet.put("leftSlidePos", pos1);
                if (pos < 3000.0 && pos1 < 3000.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    rightslide.setPower(0);
                    leftslide.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off

            }

        }
        public Action sampleLiftUp(){
            return sampleLiftUp();
        }
        public class slidesDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightslide.setPower(-1.0);
                    leftslide.setPower(-1.0);
                    initialized = true;
                }

                double pos = rightslide.getCurrentPosition();
                double pos1 = leftslide.getCurrentPosition();
                packet.put("rightSlidePos", pos);
                packet.put("leftSlidePos", pos1);
                if (pos > 100.0 && pos1 > 100.0) {
                    return true;
                } else {
                    rightslide.setPower(0);
                    leftslide.setPower(0);
                    return false;
                }
            }
        }
        public Action slidesDown(){
            return new Lifts.slidesDown();
        }
        public class slidestoSpecDrop implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rightslide.setPower(1.0);
                    leftslide.setPower(1.0);
                    initialized = true;
                }

                double pos = rightslide.getCurrentPosition();
                double pos1 = leftslide.getCurrentPosition();
                packet.put("rightSlidePos", pos);
                packet.put("leftSlidePos", pos1);
                if (pos < 3700.0 || pos1 < 3700.0) {
                    return true;
                } else {
                    rightslide.setPower(0);
                    leftslide.setPower(0);
                    sleep(1099);
                    return false;

                }
            }
        }
        public Action slidestoSpecDrop(){
            return new Lifts.slidestoSpecDrop();
        }

    }

    public class Claw{
        private Servo finger;
        private Servo outext;
        private Servo spin;
        private Servo lof;
        private Servo rof;

        public Claw(HardwareMap hardwareMap) {
            finger = hardwareMap.get(Servo.class, "finger");
            outext = hardwareMap.get(Servo.class, "outtake_ext");
            spin = hardwareMap.get(Servo.class, "spin");
            lof = hardwareMap.get(Servo.class, "left_outtake_flip");
            rof = hardwareMap.get(Servo.class, "right_outtake_flip");

        }

        public class ClawReset implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                finger.setPosition(0.1);
                sleep(100);
                rof.setPosition(1.0);
                lof.setPosition(0.0);
                outext.setPosition(0.5);
                sleep(100);
                spin.setPosition(0.33);
                return false;
            }
        }
        public Action ClawReset () {
            return new Claw.ClawReset();
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // will have to bring the claw down because it starts elevated (d_pad down)
                // will cause the arm to flip out
                finger.setPosition(0.67); // close the finger
                sleep(400);
                rof.setPosition(0.4);
                lof.setPosition(0.6);
                spin.setPosition(0.0);
                sleep(100);
                outext.setPosition(0.0);
                return false;
            }
        }
        public Action CloseClaw() {
            return new Claw.CloseClaw();
        }

        public class OpenClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                finger.setPosition(0.1);
                return false;
            }

        }
        public Action OpenClaw(){
            return new OpenClaw();
        }

        public class InitClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                finger.setPosition(0.65);
                sleep(100);
                rof.setPosition(1.0);
                lof.setPosition(0.0);
                outext.setPosition(0.5);
                sleep(100);

                return false;
            }

        }
        public Action initClaw(){
            return new InitClaw();
        }
    }

    public class intake{
        private CRServo intake;
        private Servo lif;
        private Servo lext;
        private Servo rif;
        private Servo rext;
        private ColorSensor colorSensor;


        public intake(HardwareMap hardwareMap){
            intake = hardwareMap.get(CRServo.class, "intake");
            lif = hardwareMap.get(Servo.class, "left_intake_flip");
            lext = hardwareMap.get(Servo.class, "left_extension");
            rif = hardwareMap.get(Servo.class, "right_intake_flip");
            rext = hardwareMap.get(Servo.class, "right_extension");

        }
        public class extendoMax implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rext.setPosition(0.9);
                lext.setPosition(0.00);
                sleep(300);
                lif.setPosition(0.19);
                rif.setPosition(0.81);
                while(redValue < targetValue){
                    intake.setPower(-1.0);
                }
                if (redValue >= targetValue){
                    intake.setPower(0.0);
                } // dont think i need the if statement
                return false;
            }

        }
        public Action extendoMax(){
            return new intake.extendoMax();
        }

        public class extendomin implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                lif.setPosition(0.97);
                rif.setPosition(0.03);
                sleep(300);
                rext.setPosition(0.5);
                lext.setPosition(0.4);
                intake.setPower(0.0);
                return false;
            }
        }
        public Action extendomin(){
            return new intake.extendomin();
        }

        public class extendomid implements Action{
            @Override
            public  boolean run(@NonNull TelemetryPacket packet){
                rext.setPosition(0.7);
                lext.setPosition(0.2);
                sleep(300);
                lif.setPosition(0.19);
                rif.setPosition(0.81);
                intake.setPower(-1.0);
                //sleep(1000);
                return false;
            }
        }
        public Action extendomid(){
            return new intake.extendomid();
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


    @Override
    public void runOpMode() {
        // define the poses
        initialpose = new Pose2d(9.61, -63.55, Math.toRadians(-180));
        // declaring the drivetrain
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, initialpose);

        //calling attachments
        Claw claw = new Claw(hardwareMap);
        Lifts lifts = new Lifts(hardwareMap);
        intake intake = new intake(hardwareMap);
        colorSensor = hardwareMap.get(ColorSensor.class,"color_sensor");
        getColor();
        colorTelementry();
        lifts.slideTelementry();
        //Sensors
        //colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor"); //need to do still
        //getColor();
        //colorTelementry();

        // defining each trajectory
        //goes to drop off preload
        TrajectoryActionBuilder spec1 = drivetrain.actionBuilder(initialpose)
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-41.5,Math.toRadians(-90))
                .waitSeconds(1)
                .endTrajectory();
        // goes back a little
        TrajectoryActionBuilder spec2depo = spec1.endTrajectory().fresh()
                //.splineToLinearHeading(new Pose2d(48, -48,90),0)
                .setTangent(Math.toRadians(180))
                .lineToXSplineHeading(48,Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToYConstantHeading(-50)
                .endTrajectory();
        // goes to the enxtendo position and take first blue block
        TrajectoryActionBuilder spec3depo = spec2depo.endTrajectory().fresh()
                .turnTo(Math.PI/1.5)        //turns to pick up second blue block
                .endTrajectory();
        TrajectoryActionBuilder spec4depo = spec3depo.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(53.32,-41.45,Math.toRadians(45)),0)
                .endTrajectory();
        //turns to pick up last blue block
        TrajectoryActionBuilder lastdepoforLifts = spec4depo.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(48,-48,0),Math.toRadians(90));
        TrajectoryActionBuilder spechangposition = lastdepoforLifts.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(32,-54,0),Math.toRadians(90));
        TrajectoryActionBuilder spec2drop = spechangposition.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(3.5,-35,0),Math.toRadians(-90));
        TrajectoryActionBuilder spec3drop = spechangposition.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(10,-35,0),Math.toRadians(-90));
        TrajectoryActionBuilder spec4drop = spechangposition.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12,-35,0),Math.toRadians(-90));
        TrajectoryActionBuilder park = spec4drop.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(27,-45,0),Math.toRadians(-90));

        Action traj1, traj2, traj3, traj4,traj5,traj6, traj7,traj8, traj9, traj10, trag11,trag12;
        Action a1;
        traj1 = spec1.build();
        traj2 = spec2depo.build();
        traj3 = spec3depo.build();
        traj4 = spec4depo.build();
        traj5 = lastdepoforLifts.build();
        traj6 = spechangposition.build();
        traj7 = spec2drop.build();
        traj8 = spechangposition.build();
        traj9 = spec3drop.build();
        traj10 = spechangposition.build();
        traj7 = spec4drop.build();
        traj8 = park.build();



        // sets of parallel actions
        ParallelAction p1 = new ParallelAction(
                traj1,
                lifts.SpecLiftUp()

        );

        ParallelAction p2 = new ParallelAction(
                traj2, lifts.slidesDown()
        );

        ParallelAction p3 = new ParallelAction(
                traj3, intake.extendoMax()
        );

        ParallelAction p4 = new ParallelAction(

        );

// actual segment that runs, parallel actions and sequential order
        Actions.runBlocking(claw.initClaw());
        Actions.runBlocking(lifts.initlift());
        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        claw.CloseClaw(),
                        p1,
                        lifts.slidestoSpecDrop(),
                        claw.OpenClaw(),
                        claw.ClawReset(),
                        p2

                        //traj2
                        //traj3


        )
        );
        // for run action building out everytging under a squential action and then inside of tha tput different parallel commands

    }
    public void getColor(){
        redValue = colorSensor.red();
        greenValue = colorSensor.green();
        blueValue = colorSensor.blue();
        alphaValue = colorSensor.alpha();
    }

    public void colorTelementry(){
        telemetry.addData("redValue","%.3f", redValue);
        telemetry.addData("greenValue","%.3f", greenValue);
        telemetry.addData("blueValue","%.3f", blueValue);
        telemetry.addData("alphaValue","%.3f", alphaValue);
        telemetry.update();

    }




}
