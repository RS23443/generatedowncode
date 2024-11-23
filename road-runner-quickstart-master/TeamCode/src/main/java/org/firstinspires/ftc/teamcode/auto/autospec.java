package org.firstinspires.ftc.teamcode.auto;

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

import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.Timer;
@Disabled
@Config
@Autonomous(name="4 spec", group ="Autonomous")
public class autospec extends LinearOpMode {

        /*public void liftuptospec(){
            rightslide.setTargetPosition(1000);
            leftslide.setTargetPosition(1000);
            //need to find actual numbers
       }

       public void  liftdown(){
            rightslide.setTargetPosition(0);
            leftslide.setTargetPosition(0);
       }

       public void liftuptosample(){
               rightslide.setTargetPosition(2500);
               leftslide.setTargetPosition(2500);

       }*/
        public class Lifts {
            private DcMotorEx rightslide;
            private DcMotorEx leftslide;

            public Lifts(HardwareMap hardwareMap) {
                rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
                rightslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightslide.setDirection(DcMotor.Direction.REVERSE);

                leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
                leftslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftslide.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            public class SpecLiftUp implements Action {
                // checks if the lift motor has been powered on
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
                        rightslide.setPower(-0.8);
                        leftslide.setPower(-0.8);
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
            public class slidesHalfDown implements Action {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        rightslide.setPower(-0.8);
                        leftslide.setPower(-0.8);
                        initialized = true;
                    }

                    double pos = rightslide.getCurrentPosition();
                    double pos1 = leftslide.getCurrentPosition();
                    packet.put("rightSlidePos", pos);
                    packet.put("leftSlidePos", pos1);
                    if (pos > 1500.0 && pos1 > 1500.0) {
                        return true;
                    } else {
                        rightslide.setPower(0);
                        leftslide.setPower(0);
                        return false;
                    }
                }
            }
            public Action slidesHalfDown(){
                return new Lifts.slidesHalfDown();
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
                    finger.setPosition(0.0);
                    sleep(300);
                    spin.setPosition(0.0);
                    sleep(200);
                    outext.setPosition(0.7);//find correct value for it to slam down);
                    finger.setPosition(0.17);
                    sleep(200);
                    //will be for lof and rof -> this will reset the arm inside the robot
                    rof.setPosition(0.6);
                    lof.setPosition(0.4);
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
                    finger.setPosition(0.33); // close the finger
                    sleep(400);
                    rof.setPosition(0.27);
                    lof.setPosition(0.73);
                    sleep(200);
                    outext.setPosition(0.1);//find the exact position to flip
                    sleep(500);
                    spin.setPosition(0.32);
                    return false;
                }
            }
            public Action CloseClaw() {
                return new Claw.CloseClaw();
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
                colorSensor = hardwareMap.get(ColorSensor.class,"color_sensor");
                getColor();
                colorTelementry();
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
        public Pose2d spec1pose;
        public Pose2d back;
        public Pose2d extendopose;
        public Pose2d spechang;
        public Pose2d spec2pose;
        public Pose2d spec3pose;
        public Pose2d spec4pose;
        public Pose2d t1;
        public Pose2d t2;
        public Pose2d specXpose;

    //color sensor Values
        public double redValue;
        public double greenValue;
        public double blueValue;
        public double alphaValue;
        public double targetValue = 1000;


        @Override
        public void runOpMode() {
            // define the poses
            initialpose = new Pose2d(-10, 66, Math.toRadians(90));
            spec1pose = new Pose2d(-5, 30, Math.toRadians(90));
            back = new Pose2d(-5,37,Math.toRadians(90));
            extendopose = new Pose2d(-48, 48, Math.toRadians(270));
            spechang = new Pose2d(-35,60, Math.toRadians(270));
            spec2pose = new Pose2d(-7, 30, Math.toRadians(270));
            spec3pose =  new Pose2d(-10, 30, Math.toRadians(270));
            spec4pose = new Pose2d(-12,30,Math.toRadians(270));
            //parkpose = new Pose2d(-50,60, Math.toRadians(270));

            t1 = new Pose2d(-48,48,Math.toRadians(270-21.6));
            t2 = new Pose2d(-48,48,Math.toRadians(270-40));




            // declaring the drivetrain
            MecanumDrive drivetrain = new MecanumDrive(hardwareMap, initialpose);

            //calling attachments
            Claw claw = new Claw(hardwareMap);
            Lifts lifts = new Lifts(hardwareMap);
            intake intake = new intake(hardwareMap);

            //Sensors
            //colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor"); //need to do still
            //getColor();
            //colorTelementry();

            // defining each trajectory
            //goes to drop off preload
            TrajectoryActionBuilder spec1 = drivetrain.actionBuilder(initialpose)
                    .splineTo(new Vector2d(-5,30), Math.toRadians(90));
            // goes back a little
            TrajectoryActionBuilder backs = spec1.endTrajectory().fresh()
                    .splineTo(new Vector2d(-5, 37), Math.toRadians(90));
            // goes to the enxtendo position and take first blue block
            TrajectoryActionBuilder spec2depo = backs.endTrajectory().fresh()
                    .splineTo(new Vector2d(-48, 48), Math.toRadians(270));
            //turns to pick up second blue block
            TrajectoryActionBuilder spec3depo = spec2depo.endTrajectory().fresh()
                    .turnTo(270-21.6);
            //turns to pick up last blue block
            TrajectoryActionBuilder spec4depo = spec3depo.endTrajectory().fresh()
                    .splineTo(new Vector2d(-48, 48), Math.toRadians(270-40));
            //goes to where the speciemens are being hanged
            TrajectoryActionBuilder specpickup = spec4depo.endTrajectory().fresh()
                    .splineTo(new Vector2d(-35, 60), Math.toRadians(270));
            // takes the first of three depod specimens and hangs it up
            TrajectoryActionBuilder spec2drop = specpickup.endTrajectory().fresh()
                    .splineTo(new Vector2d(-7,30), Math.toRadians(90));
            //takes second of three depod specs and hnags it up
            TrajectoryActionBuilder spec3drop = spec2drop.endTrajectory().fresh()
                    .splineTo(new Vector2d(-10,30), Math.toRadians(90));
            // takes the third of three depod specs and hangs it up
            TrajectoryActionBuilder spec4drop = spec3drop.endTrajectory().fresh()
                    .splineTo(new Vector2d(-12,30), Math.toRadians(90));
            TrajectoryActionBuilder park = spec4drop.endTrajectory().fresh()
                    .splineTo(new Vector2d(-50,60), Math.toRadians(270));

            //pushing trajectories into Actions
            Action traj1,traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9, traj10;
            traj1 = spec1.build();
            traj2 =backs.build();
            traj3 =spec2depo.build();
            traj4 =spec3depo.build();
            traj5 =spec4depo.build();
            traj6 =specpickup.build();
            traj7 =spec2drop.build();
            traj8 = spec3drop.build();
            traj9 = spec4drop.build();
            traj10 = park.build();




            // sets of parallel actions
            ParallelAction p1 = new ParallelAction(
                    traj1,lifts.SpecLiftUp()
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
            Actions.runBlocking(claw.CloseClaw());
            waitForStart();
            Timer.start();
            Actions.runBlocking(
                    new SequentialAction(
                            p1



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

