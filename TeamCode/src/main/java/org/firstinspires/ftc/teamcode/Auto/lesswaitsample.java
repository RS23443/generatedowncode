package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.Robot_Controller;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.deposit_subsystem;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.intake_subsystem;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.lifts_subsystem;

import org.firstinspires.ftc.teamcode.MecanumDrive;
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
@Autonomous(name="4 sample with less wait", group ="Autonomous")
public class lesswaitsample extends LinearOpMode {


    public class Lifts {
        //private DcMotorEx rightslide;
        //private DcMotorEx leftslide;
        private lifts_subsystem liftss;

        public Lifts(HardwareMap hardwareMap) {
            this.liftss = new lifts_subsystem(hardwareMap,"leftslide","rightslide");

            /*rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
            leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");

            leftslide.setDirection((DcMotorEx.Direction.REVERSE));

            leftslide.setMode((DcMotorEx.RunMode.STOP_AND_RESET_ENCODER));
            rightslide.setMode((DcMotorEx.RunMode.STOP_AND_RESET_ENCODER));
            leftslide.setTargetPosition(0);
            rightslide.setTargetPosition(0);

            leftslide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rightslide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            rightslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            ((DcMotorEx) rightslide).setVelocity(2000);
            ((DcMotorEx) leftslide).setVelocity(2000);*/

            telemetry.addData("leftslide", liftss.getLeftSlidePosition());
            telemetry.addData("rightslide", liftss.getRightSlidePosition());
            telemetry.update();

        }
        public class MoveLiftToPosition implements Action {

            private final lifts_subsystem liftSubsystem;
            private final int targetPosition;


            //public MoveLiftToPosition(lifts_subsystem liftSubsystem, int targetPosition) {
            public MoveLiftToPosition(lifts_subsystem liftSubsystem, int targetPosition){
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
                telemetry.addData("right",currentRight);
                telemetry.addData("left",currentLeft);
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
                double ticksPerInch = ((4498+4345)/2)/44.5;

                this.power = power;
                this.ticksToMove = (int) (inches * ticksPerInch);

                liftss.stopandreset();
                liftss.liftrunencoders();


                this.endTime = System.currentTimeMillis() + secondsToWait * 1000;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (System.currentTimeMillis() < endTime)
                {
                    return true;
                }

                if (Math.abs(liftss.getLeftSlidePosition()) < ticksToMove){ //&& Math.abs(getRightSlidePosition()) < ticksToMove) {
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


        /*public class Initifts implements Action {

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

        public class sampleparallelLiftUp implements Action {
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
                if (pos < 2000.0 && pos1 < 2000.0) {
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
        public Action sampleparallelLiftUp(){
            return new sampleparallelLiftUp();
        }

        public class samplefullLiftUp implements Action {
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
                if (pos < 3800 && pos1 < 3800) {
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
        public Action samplefullLiftUp(){
            return new samplefullLiftUp();
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
                if (pos > 400.0 && pos1 > 400.0) {
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
        }*/

    }

    /*public class Claw{
        private deposit_subsystem depo;
        private intake_subsystem intake_sub;
        private Servo finger;
        private Servo outext;
        private Servo spin;
        private Servo lof;
        private Servo rof;

        public Claw(HardwareMap hardwareMap) {
            intake_sub = new intake_subsystem(hardwareMap,"left_intake_flip","right_intake_flip","left_extension","right_extension","clawtakespin","clawtake");
            depo = new deposit_subsystem(intake_sub, hardwareMap,"left_outtake_flip", "right_outtake_flip","outtake_ext","spin","finger");
            /*finger = hardwareMap.get(Servo.class, "finger");
            outext = hardwareMap.get(Servo.class, "outtake_ext");
            spin = hardwareMap.get(Servo.class, "spin");
            lof = hardwareMap.get(Servo.class, "left_outtake_flip");
            rof = hardwareMap.get(Servo.class, "right_outtake_flip");*/

        /*}

        public class ClawReset implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                finger.setPosition(0.0);
                sleep(100);
                rof.setPosition(0.73);
                lof.setPosition(0.27);
                outext.setPosition(0.5);
                sleep(100);
                spin.setPosition(0.00);
                return false;
            }
        }
        public Action ClawReset () {
            return new Claw.ClawReset();
        }
        public class fingerClose implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                finger.setPosition(0.67);
                return false;
            }

        }
        public Action fingerclose(){
            return new Claw.fingerClose();
        }

        public class FlipClawHalf implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // will have to bring the claw down because it starts elevated (d_pad down)
                // will cause the arm to flip out
                //finger.setPosition(0.67); // close the finger
                sleep(400);
                rof.setPosition(0.35);
                lof.setPosition(0.65);
                spin.setPosition(0.0);
                sleep(100);
                outext.setPosition(0.0);
                return false;
            }
        }
        public Action flipclawhalf() {
            return new Claw.FlipClawHalf();
        }

        public class dumpOff implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                finger.setPosition(0.65); // close the finger
                rof.setPosition(0.25);
                lof.setPosition(0.75);
                spin.setPosition(0.0);
                sleep(100);
                outext.setPosition(0.0);
                return false;
            }
        }

        public Action dumpoff(){
            return new Claw.dumpOff();
        }

        public class OpenClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                finger.setPosition(0.0);
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
                rof.setPosition(0.73);
                lof.setPosition(0.27);
                outext.setPosition(0.5);
                sleep(100);
                spin.setPosition(0.00);

                return false;
            }

        }
        public Action initClaw(){
            return new InitClaw();
        }
    }



    /*public class intake{
        private intake_subsystem intake_sub;
        //private Servo intake;
        //private Servo intakespin;
        //private Servo lif;
        //private Servo lext;
        //private Servo rif;
        //private Servo rext;
        //private ColorSensor colorSensor;


        public intake(HardwareMap hardwareMap){
            /*intake = hardwareMap.get(Servo.class, "clawtake");
            intakespin = hardwareMap.get(Servo.class,"clawtakespin");
            lif = hardwareMap.get(Servo.class, "left_intake_flip");
            lext = hardwareMap.get(Servo.class, "left_extension");
            rif = hardwareMap.get(Servo.class, "right_intake_flip");
            rext = hardwareMap.get(Servo.class, "right_extension");*/

    //intake_sub = new intake_subsystem(hardwareMap,"left_intake_flip","right_intake_flip","left_extension","right_extension","clawtakespin","clawtake");

    //}
    //public class extendoMax implements Action{
    //  @Override
    //public boolean run(@NonNull TelemetryPacket packet) {
    //intake.setPosition(0.1);
    //intakespin.setPosition(0.3);
    //rext.setPosition(0.8);
    //lext.setPosition(0.10);
    //sleep(300);
    //lif.setPosition(0.1);
    //rif.setPosition(0.9);
    //  try {
    //    intake_sub.maxextension();
    // } catch (InterruptedException e) {
    //   throw new RuntimeException(e);
    // }
    //return false;
    //}

    //}
    //public Action extendoMax(){
    //  return new intake.extendoMax();
    //}

//        public class extendoMaxfor3rd implements Action{
//            @Override
  /*          public boolean run(@NonNull TelemetryPacket packet) {
                /*intake.setPosition(0.6);
                intakespin.setPosition(0.3);
                sleep(100);
                lif.setPosition(0.3);
                rif.setPosition(0.7);
                sleep(100);
                rext.setPosition(0.8);
                lext.setPosition(0.10);
                sleep(300);
                lif.setPosition(0.1);
                rif.setPosition(0.9);*/
               /* try {
                    intake_sub.max_ext_for_3rd();
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }

        }
        public Action extendoMaxfor3rd(){
            return new intake.extendoMaxfor3rd();}

        public class transferpose implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                /*intake.setPosition(0.1);
                intakespin.setPosition(0.0);
                sleep(200);
                lif.setPosition(1.0);
                rif.setPosition(0.0);
                sleep(200);
                rext.setPosition(0.5);
                lext.setPosition(0.4);*/
 /*               try {
                    intake_sub.transferpose();
                } catch (InterruptedException e){
                    throw new RuntimeException(e);
                }
                return false;
            }
        }
        public Action transferpose(){
            return new intake.transferpose();
        }


        /*public class extendominfor3rd implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //intake.setPosition(0.1);
                intake.setPosition(0.6);
                intakespin.setPosition(0.0);
                sleep(200);
                lif.setPosition(1.0);
                rif.setPosition(0.0);
                intake.setPosition(0.1);
                sleep(200);
                rext.setPosition(0.5);
                lext.setPosition(0.4);

                return false;
            }
        }
        public Action extendominfor3rd(){
            return new intake.extendominfor3rd();
        }*/

        /*public class extendomid implements Action{
            @Override
            public  boolean run(@NonNull TelemetryPacket packet){
                intakespin.setPosition(0.3);
                rext.setPosition(0.71);
                lext.setPosition(0.19);
                sleep(300);
                lif.setPosition(0.1);
                rif.setPosition(0.9);
                //sleep(1000);
                return false;
            }
        }
        public Action extendomid(){
            return new intake.extendomid();
        }*/

       /* public class intakeclose implements Action{
            @Override
            public  boolean run(@NonNull TelemetryPacket packet){
                intake_sub.intakeclose();
                return false;
            }
        }
        public Action intakeclose(){
            return new intakeclose();
        }

        public class intakeopen implements Action{
            @Override
            public  boolean run(@NonNull TelemetryPacket packet){
                intake_sub.intakeopen();
                return false;
            }

        }

        public Action intakeopen() {
            return new intake.intakeopen();
        }

        public class intakespin implements Action{
            @Override
            public  boolean run(@NonNull TelemetryPacket packet){
                try {
                    intake_sub.intakespinvertical();
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }

        }
        public Action intakespin(){
            return new intake.intakespin();
        }

    }*/

    public class Robot_controller{

        private Robot_Controller attachments;
        public Robot_controller(intake_subsystem intake_sub, deposit_subsystem depo_sub){
            this.attachments = new Robot_Controller(intake_sub,depo_sub);
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
        public Action intake_ext_max(){
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
        public Action intake_ext_max_3rd(){
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
        public Action transfer(){
            return new transferpose();
        }
        public class intake_open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                attachments.int_open();
                return false;
            }
        }
        public Action intake_open(){
            return new intake_open();
        }

        public class intake_close implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                attachments.int_close();
                return false;
            }
        }
        public Action intake_close(){
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
        public Action int_vert(){
            return new int_spin_vert();
        }

        public class int_spin_hor implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                attachments.int_horizontal_spin();
                return false;
            }
        }
        public Action int_hor(){
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
        public Action preint(){
            return new finger_close_pose();
        }

        public class droptake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                attachments.dropintake();
                return false;
            }
        }
        public Action droptake(){
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
        public Action depo_grab(){
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
        public Action depo_half(){
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
        public Action depo_dump(){
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
        public Action depo_down(){
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
        public Action depo_preload(){
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
        public Action fing_open(){
            return new fing_open_pose();
        }

        public class finger_close_pose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                attachments.fingerclose();
                return false;
            }
        }
        public Action finger_close(){
            return new finger_close_pose();
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
        initialpose = new Pose2d(-14.5, -64, Math.toRadians(0));
        // declaring the drivetrain
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, initialpose);

        //calling attachments
        Lifts lifts = new Lifts(hardwareMap);
        //needed to intialize the robot controller
        intake_subsystem intake_sub = new intake_subsystem(hardwareMap,"left_intake_flip","right_intake_flip","left_extension","right_extension","clawtakespin","clawtake");
        deposit_subsystem depo_sub = new deposit_subsystem(hardwareMap,"left_outtake_flip", "right_outtake_flip","outtake_ext","spin","finger");
        Robot_controller servo_attachments = new Robot_controller(intake_sub, depo_sub);

        //intake intake = new intake(hardwareMap);
        //Claw depo = new Claw(hardwareMap);

        colorSensor = hardwareMap.get(ColorSensor.class,"color_sensor");
        getColor();
        colorTelementry();



        // defining each trajectory
        //goes to drop off preload
        TrajectoryActionBuilder preload = drivetrain.actionBuilder(initialpose)
                .setTangent(Math.toRadians(90))
                .lineToY(-56)
                .setTangent(0)
                .lineToXLinearHeading(-55,Math.toRadians(45.6557))
                .endTrajectory();
        TrajectoryActionBuilder picksample1 = preload.endTrajectory().fresh()
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-55,-46.5,Math.toRadians(98)),90)
                //.setTangent(0)
                //.lineToXLinearHeading(-56,Math.toRadians(98))
                .stopAndAdd(servo_attachments.intake_ext_max())
                .waitSeconds(0.1)
                .stopAndAdd(servo_attachments.transfer())
                .waitSeconds(0.1)
                .stopAndAdd(servo_attachments.depo_grab())
                .waitSeconds(0.1)
                .stopAndAdd(servo_attachments.depo_half())
                .endTrajectory();
        TrajectoryActionBuilder dropsample1 = picksample1.endTrajectory().fresh()
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-55,-53,Math.toRadians(45)),90)
                //.lineToYLinearHeading(-55,Math.toRadians(45))
                .endTrajectory();
        TrajectoryActionBuilder picksample2 = dropsample1.endTrajectory().fresh()
                .stopAndAdd(servo_attachments.intake_open())
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-53.5,-44.5,Math.toRadians(74)),0)
                .stopAndAdd(servo_attachments.intake_ext_max())
                .waitSeconds(0.1)
                .stopAndAdd(servo_attachments.transfer())
                .waitSeconds(0.1)
                .stopAndAdd(servo_attachments.depo_grab())
                .waitSeconds(0.1)
                .stopAndAdd(servo_attachments.depo_half())
                .endTrajectory();
        //.turnto
        TrajectoryActionBuilder dropsample2 = picksample2.endTrajectory().fresh()
                .waitSeconds(0.1)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-53.5,-52.5, Math.toRadians(45)),0)
                .endTrajectory();
        TrajectoryActionBuilder picksample3parallel = dropsample2.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-47,-27,Math.toRadians(-180)),0)
                .stopAndAdd(servo_attachments.intake_ext_max_3rd())
                .waitSeconds(0.1)
                .stopAndAdd(servo_attachments.intake_close())
                .waitSeconds(0.1)
                .lineToXLinearHeading(-44,Math.toRadians(-180))
                .stopAndAdd(servo_attachments.transfer())
                .waitSeconds(0.1)
                .stopAndAdd(servo_attachments.depo_grab())
                .waitSeconds(0.1)
                .stopAndAdd(servo_attachments.depo_half())
                .endTrajectory();
        TrajectoryActionBuilder dropsample3 = picksample3parallel.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-53,-52.5, Math.toRadians(45)),0)
                .endTrajectory();
        TrajectoryActionBuilder middlesample_park = dropsample3.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-29,-10,Math.toRadians(0)),0)
                .endTrajectory();
        // goes back a little

        Action traj1, traj2, traj3,traj4,traj5,traj6, traj7,traj8;
        traj1 = preload.build();
        traj2 = picksample1.build();
        traj3 = dropsample1.build();
        traj4 = picksample2.build();
        traj5 = dropsample2.build();
        traj6= picksample3parallel.build();
        traj7= dropsample3.build();
        traj8 = middlesample_park.build();




        // sets of parallel actions
        ParallelAction p1 = new ParallelAction(
                traj1,
                servo_attachments.depo_preload(),
                lifts.liftUp(1.0, 36, 0)
        );
        ParallelAction p2 = new ParallelAction(
                traj2
        );

        ParallelAction p3 = new ParallelAction(
                traj3,
                servo_attachments.depo_half(),
                lifts.liftUp(1.0,36,0)
        );

        ParallelAction p4 = new ParallelAction(
                traj4,
                servo_attachments.depo_grab()
        );
        ParallelAction p5 = new ParallelAction(
                traj5,
                servo_attachments.depo_half(),
                lifts.liftUp(1.0,36,0)
        );

        ParallelAction p6 = new ParallelAction(
                traj6,
                servo_attachments.depo_grab()
        );
        ParallelAction p7 = new ParallelAction(
                traj7,
                servo_attachments.depo_half(),
                lifts.liftUp(1.0,36,0)
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

                        servo_attachments.depo_grab(),
                        lifts.moveLiftAction(0),


                        p2,
                        servo_attachments.depo_half(),

                        p3,
                        servo_attachments.depo_dump(),
                        servo_attachments.fing_open(),
                        servo_attachments.depo_grab(),
                        lifts.moveLiftAction(0),

                        p4,
                        servo_attachments.depo_half(),

                        p5,
                        servo_attachments.depo_dump(),
                        servo_attachments.fing_open(),
                        servo_attachments.depo_grab(),
                        lifts.moveLiftAction(0),

                        p6,

                        p7,
                        servo_attachments.depo_dump(),
                        servo_attachments.fing_open(),
                        servo_attachments.depo_grab(),

                        p8
                        //lifts.moveLiftAction(0),

                        //traj8


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
