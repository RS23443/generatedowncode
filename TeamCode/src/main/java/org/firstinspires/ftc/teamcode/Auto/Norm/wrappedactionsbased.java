package org.firstinspires.ftc.teamcode.Auto.Norm;

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

import org.firstinspires.ftc.teamcode.ModulesToImport.Conditionals.RedOrBlueCondition;
import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.ColorBasedActionWrapper;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.Robot_Controller;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.deposit_subsystem;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.intake_subsystem;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.lifts_subsystem;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;
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
@Autonomous(name="4 sample conditional", group ="Autonomous")
public class wrappedactionsbased extends LinearOpMode {


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
    public ColorSensor rightcolor;
    public ColorSensor leftcolor;
    public ColorSensor clawcolor;
    //init the poses
    public Pose2d initialpose;


    //color sensor Values
    public double rightredValue;
    public double rightgreenValue;
    public double rightblueValue;
    public double rightalphaValue;

    public double leftredValue;
    public double leftgreenValue;
    public double leftblueValue;
    public double leftalphaValue;
    public double clawredValue;
    public double clawgreenValue;
    public double clawblueValue;
    public double clawalphaValue;
    public double targetValue = 1000;


    @Override
    public void runOpMode() {
        // define the poses
        initialpose = new Pose2d(-36, -63.5, Math.toRadians(0));
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

        rightcolor = hardwareMap.get(ColorSensor.class,"color_sensor");
        getColorright();
        getColorleft();
        getColorclaw();
        colorTelementry();



        // defining each trajectory
        //goes to drop off preload
        TrajectoryActionBuilder preload = drivetrain.actionBuilder(initialpose)
                .setTangent(Math.toRadians(90))
                .lineToY(-56)
                .setTangent(0)
                .lineToXLinearHeading(-54,Math.toRadians(45.6557))
                .endTrajectory();
        TrajectoryActionBuilder picksample1 = preload.endTrajectory().fresh()
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-55,-48,Math.toRadians(90)),90)
                //.setTangent(0)
                //.lineToXLinearHeading(-56,Math.toRadians(98))
                .stopAndAdd(servo_attachments.intake_ext_max())
                .waitSeconds(0.3)
                .stopAndAdd(servo_attachments.transfer())
                .waitSeconds(0.2)
                .stopAndAdd(servo_attachments.depo_grab())
                .waitSeconds(0.2)
                .stopAndAdd(servo_attachments.depo_half())
                .endTrajectory();
        TrajectoryActionBuilder dropsample1 = picksample1.endTrajectory().fresh()
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-54.5,-56,Math.toRadians(45)),90)
                //.lineToYLinearHeading(-55,Math.toRadians(45))
                .endTrajectory();
        TrajectoryActionBuilder picksample2 = dropsample1.endTrajectory().fresh()
                .stopAndAdd(servo_attachments.intake_open())
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-52.2,-45.3,Math.toRadians(70)),0)
                .stopAndAdd(servo_attachments.intake_ext_max())
                .waitSeconds(0.2)
                .stopAndAdd(servo_attachments.transfer())
                .waitSeconds(0.2)
                .stopAndAdd(servo_attachments.depo_grab())
                .waitSeconds(0.2)
                .stopAndAdd(servo_attachments.depo_half())
                .endTrajectory();
        //.turnto
        TrajectoryActionBuilder dropsample2 = picksample2.endTrajectory().fresh()
                .waitSeconds(0.2)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-54.5,-56, Math.toRadians(45)),0)
                .endTrajectory();
        TrajectoryActionBuilder picksample3parallel = dropsample2.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-41.5,-27,Math.toRadians(-180)),0)
                .stopAndAdd(servo_attachments.intake_ext_max_3rd())
                .waitSeconds(0.3)
                .stopAndAdd(servo_attachments.intake_close())
                .waitSeconds(0.2)
                .lineToXLinearHeading(-40,Math.toRadians(-180))
                .stopAndAdd(servo_attachments.transfer())
                .waitSeconds(0.2)
                .stopAndAdd(servo_attachments.depo_grab())
                .waitSeconds(0.2)
                .stopAndAdd(servo_attachments.depo_half())
                .endTrajectory();
        TrajectoryActionBuilder dropsample3 = picksample3parallel.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-54.5,-56, Math.toRadians(45)),0)
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

        List<Action> trajectories = new ArrayList<>();
        trajectories.add(traj3);
        trajectories.add(traj5);
        trajectories.add(traj7);

        List<Action> wrappedActions = ColorBasedActionWrapper.wrapAllWithColorCheck(
                trajectories,
                leftcolor,
                rightcolor,
                new RedOrBlueCondition(),
                "blue",
                drivetrain
        );


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
                wrappedActions.get(0),
                servo_attachments.depo_half(),
                lifts.liftUp(1.0,36,0)
        );

        ParallelAction p4 = new ParallelAction(
                traj4,
                servo_attachments.depo_grab()
        );
        ParallelAction p5 = new ParallelAction(
                wrappedActions.get(1),
                servo_attachments.depo_half(),
                lifts.liftUp(1.0,36,0)
        );

        ParallelAction p6 = new ParallelAction(
                traj6,
                servo_attachments.depo_grab()
        );
        ParallelAction p7 = new ParallelAction(
                wrappedActions.get(2),
                servo_attachments.depo_half(),
                lifts.liftUp(1.0,36,0)
        );
        ParallelAction p8 = new ParallelAction(
                lifts.moveLiftAction(0)//,
                //traj8
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
                        lifts.moveLiftAction(300),


                        p2,
                        servo_attachments.depo_half(),

                        p3,
                        servo_attachments.depo_dump(),
                        servo_attachments.fing_open(),
                        servo_attachments.depo_grab(),
                        lifts.moveLiftAction(300),

                        p4,
                        servo_attachments.depo_half(),

                        p5,
                        servo_attachments.depo_dump(),
                        servo_attachments.fing_open(),
                        servo_attachments.depo_grab(),
                        lifts.moveLiftAction(300),

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
        // for run action building out everything under a sequential action and then inside of that put different parallel commands

    }
    public void getColorright(){
        rightredValue = rightcolor.red();
        rightgreenValue = rightcolor.green();
        rightblueValue = rightcolor.blue();
        rightalphaValue = rightcolor.alpha();
    }

    public void getColorleft(){
        leftredValue = leftcolor.red();
        leftgreenValue = leftcolor.green();
        leftblueValue = leftcolor.blue();
        leftalphaValue = leftcolor.alpha();
    }

    public void getColorclaw(){
        clawredValue = clawcolor.red();
        clawgreenValue = clawcolor.green();
        clawblueValue = clawcolor.blue();
        clawalphaValue = clawcolor.alpha();
    }

    public void colorTelementry(){
        telemetry.addData("rightredValue","%.3f", rightredValue);
        telemetry.addData("rightgreenValue","%.3f", rightgreenValue);
        telemetry.addData("rightblueValue","%.3f", rightblueValue);
        telemetry.addData("rightalphaValue","%.3f", rightalphaValue);
        telemetry.addData("leftredValue","%.3f", leftredValue);
        telemetry.addData("leftgreenValue","%.3f", leftgreenValue);
        telemetry.addData("leftblueValue","%.3f", leftblueValue);
        telemetry.addData("leftalphaValue","%.3f", leftalphaValue);
        telemetry.addData("clawredValue","%.3f", clawredValue);
        telemetry.addData("clawgreenValue","%.3f", clawgreenValue);
        telemetry.addData("clawblueValue","%.3f", clawblueValue);
        telemetry.addData("clawalphaValue","%.3f", clawalphaValue);
        telemetry.update();

    }





}
