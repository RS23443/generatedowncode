package org.firstinspires.ftc.teamcode.auto;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import java.util.Vector;

@Config
@Autonomous(name="1 spec + 3 sample", group ="Autonomous")
public class autov1 extends LinearOpMode {

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
            return new SpecLiftUp();
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
            return new slidesDown();
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
            return new slidesHalfDown();
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
            return new ClawReset();
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
            return new CloseClaw();
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
            return new extendoMax();
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
            return new extendomin();
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
            return new extendomid();
        }
    }
    public MecanumDrive drivetrain;
    public ColorSensor colorSensor;
    //init the poses
    public Pose2d initialpose;
    public Pose2d spec1pose;
    public Pose2d sample1pose;
    public Pose2d sampledroppose;
    public Pose2d sample2pose;
    public Pose2d sample2droppose;
    public Pose2d sample3pose;
    public Pose2d sample3droppose;

    //color sensor Values
    public double redValue;
    public double greenValue;
    public double blueValue;
    public double alphaValue;
    public double targetValue = 1000;


    @Override
    public void runOpMode() {
        // define the poses
        initialpose = new Pose2d(10, 66, Math.toRadians(90));
        spec1pose = new Pose2d(10, 50, Math.toRadians(90));
        sample1pose = new Pose2d(50, 50, Math.toRadians(90));
        sampledroppose = new Pose2d(60, 60, Math.toRadians(60));
        sample2pose = new Pose2d(62, 50, Math.toRadians(90));
        sample2droppose =  new Pose2d(60, 60, Math.toRadians(60));
        sample3pose = new Pose2d(70,50,Math.toRadians(90));
        sample3droppose = new Pose2d(60,60,Math.toRadians(90));


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
        TrajectoryActionBuilder spec1 = drivetrain.actionBuilder(initialpose)
                .splineTo(new Vector2d(10,24), Math.toRadians(270))//wrong code
                .waitSeconds(2);
        TrajectoryActionBuilder sample1 = drivetrain.actionBuilder(spec1pose)
                .splineTo(new Vector2d(50, 50), Math.toRadians(90));
        TrajectoryActionBuilder sampledrop = drivetrain.actionBuilder(sample1pose)
                .splineTo(new Vector2d(60, 60), Math.toRadians(60));
        TrajectoryActionBuilder sample2 = drivetrain.actionBuilder(sampledroppose)
                .splineTo(new Vector2d(62, 50), Math.toRadians(90));
        TrajectoryActionBuilder sampledrop2 = drivetrain.actionBuilder(sample2pose)
                .splineTo(new Vector2d(60, 60), Math.toRadians(60));
        TrajectoryActionBuilder sample3 = drivetrain.actionBuilder(sample2droppose)
                .splineTo(new Vector2d(60, 60), Math.toRadians(60));
        TrajectoryActionBuilder sample3drop= drivetrain.actionBuilder(sample3pose)
                .splineTo(new Vector2d(70,50), Math.toRadians(90));
        TrajectoryActionBuilder park = drivetrain.actionBuilder(sample3droppose)
                .splineTo(new Vector2d(32,18), Math.toRadians(0));

        //pushing trajectories into Actions
        Action traj1,traj2, traj3, traj4, traj5, traj6, traj7, traj8;
        traj1 = spec1.build();
        traj2 =sample1.build();
        traj3 =sampledrop.build();
        traj4 =sample2.build();
        traj5 =sampledrop2.build();
        traj6 =sample3.build();
        traj7 =sample3drop.build();
        traj8 = park.build();

// actual segment that runs, parallel actions and sequential order
        Actions.runBlocking(claw.CloseClaw());
        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                traj1,
                                lifts.SpecLiftUp()),

                        new ParallelAction(
                        lifts.slidesHalfDown(),
                        claw.ClawReset()),

                        new ParallelAction(
                                lifts.slidesDown(),
                                traj2),

                        intake.extendoMax(),
                        intake.extendomin(),
                        claw.CloseClaw(),

                        new ParallelAction(
                                traj3,
                                lifts.sampleLiftUp()),

                        claw.ClawReset(),

                        new ParallelAction(
                                traj4,
                                lifts.slidesDown(),
                                intake.extendoMax()),

                        intake.extendomin(),
                        claw.CloseClaw(),

                        new ParallelAction(
                                traj5,
                                lifts.sampleLiftUp()),

                        claw.ClawReset(),

                        new ParallelAction(
                                traj6,
                                lifts.slidesDown(),
                                intake.extendoMax()),

                        intake.extendomin(),
                        claw.CloseClaw(),

                        new ParallelAction(
                                traj7,
                                lifts.sampleLiftUp()),
                        claw.ClawReset(),

                        new ParallelAction(
                                lifts.slidesDown(),
                                traj8
                        )
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



