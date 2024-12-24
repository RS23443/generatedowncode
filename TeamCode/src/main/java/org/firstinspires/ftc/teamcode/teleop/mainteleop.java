package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@TeleOp(name="main", group="LinearOpMode")
public class mainteleop extends LinearOpMode {
    // wifi name: 23443-RC, password:RoboSupport23443
    private PIDController controller;
    private PIDController controller1;
    public static double p= 0.03, i=0, d=0.0006;
    public static double p1 = 0.03,i1 = 0,d1 = 0.0006;
    public static double f = 0.01;
    public static double f1 = 0.01;
    public static int target = 0;
    private final double ticks_in_degree = 537.7 / 180.0;
    public DcMotorEx frmotor; // control hub 2
    public DcMotorEx brmotor; // control hub 3
    public DcMotorEx flmotor; // control hub 0
    public DcMotorEx blmotor; //control hub 1
    private DcMotorEx leftslide; //exapnsion hub 0
    private DcMotorEx rightslide; //expansion hub 1

    public Servo intake; // control hub servo 0
    public Servo lif; // control hub servo 1
    public Servo lext; // control hub servo 2
    public Servo lof; // control hub servo 3
    public Servo finger; // control hub servo 4
    public Servo rif; //expansion hub servo 0
    public Servo rext; // expansion hub servo 1
    public Servo rof; // expansion hub servo 2

    public Servo outext; //expansion hub servo 3
    public Servo spin; // expansion hub 4
    public Servo intakespin;//expansion hub 5

    private ColorSensor colorSensor;
    private double redValue;
    private double greenValue;
    private double blueValue;
    private double alphaValue;
    private double targetValue = 1000;

    //public static int pos = 0;
    public double x;
    public double rx;
    public double y;
    public double ry;
    public int count;


    public double denominator;
    public double speed = 1.0;


    @Override
    public void runOpMode() throws InterruptedException {
        //controller = new PIDController(p, i, d);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //controller1 = new PIDController(p1,i1,d1);

        //Servos
        leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
        intake = hardwareMap.get(Servo.class, "clawtake"); // this is the axon - do the spin
        lif = hardwareMap.get(Servo.class,"left_intake_flip");
        lext = hardwareMap.get(Servo.class, "left_extension");
        lof = hardwareMap.get(Servo.class, "left_outtake_flip");
        finger = hardwareMap.get(Servo.class, "finger");
        rif = hardwareMap.get(Servo.class, "right_intake_flip");
        rext = hardwareMap.get(Servo.class,"right_extension");
        rof = hardwareMap.get(Servo.class, "right_outtake_flip");
        outext = hardwareMap.get(Servo.class, "outtake_ext");
        spin = hardwareMap.get(Servo.class, "spin");
        intakespin = hardwareMap.get(Servo.class,"clawtakespin"); //thia ia thw speed servo so the claw

        //Sensors
        //colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor"); //need to do still
        //getColor();
        //colorTelementry();

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


        // localizer
        Pose2d startingpose = new Pose2d(30,30,30);
        MecanumDrive drive = new MecanumDrive(hardwareMap,startingpose);

        //for encoders reverse left
        waitForStart();
        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            telemetry.addData("estimated pose", drive.updatePoseEstimate());
            //getColor();
            //colorTelementry();
            if (gamepad2.left_bumper){
               //min
                finger.setPosition(0.0);
                sleep(100);
                rof.setPosition(0.83); // need to update code to implement the changes
                lof.setPosition(0.17);
                outext.setPosition(0.9);
                sleep(100);
                spin.setPosition(0.00);

            }

            if (gamepad2.right_bumper){
                //max
                // will have to bring the claw down because it starts elevated (d_pad down)
                // will cause the arm to flip out
                finger.setPosition(0.65); // close the finger
                sleep(200);
                intakespin.setPosition(0.3);
                sleep(200);
                rof.setPosition(0.45);
                lof.setPosition(0.55);
                spin.setPosition(0.0);
                sleep(100);
                outext.setPosition(0.0);

            }
            if(gamepad2.y){
                finger.setPosition(0.0);
                intakespin.setPosition(0.3);

            }

            if(gamepad2.b){
                finger.setPosition(0.0);
                sleep(100);
                rof.setPosition(0.7); // need to update code to implement the changes
                lof.setPosition(0.3);
                outext.setPosition(0.9);
                sleep(100);
                spin.setPosition(0.00);
            }


            if (gamepad1.left_bumper){
                //verical
                    intake.setPosition(0.6);
                    lif.setPosition(0.08);
                    rif.setPosition(0.92);



                }



            if (gamepad1.a) {
                intakespin.setPosition(0.3); // opens claw
                intake.setPosition(0.1);
                sleep(50);
                // sets extensions to max
                rext.setPosition(0.9);
                lext.setPosition(0.00);
                sleep(200);
                lif.setPosition(0.11);
                rif.setPosition(0.89);

            }
            if (gamepad1.b) {
                //should set extensions to minimum
                intakespin.setPosition(0.0);
                sleep(100);
                intake.setPosition(0.1);
                sleep(200);
                lif.setPosition(1);
                rif.setPosition(0.0);
                sleep(200);
                rext.setPosition(0.5);
                lext.setPosition(0.4);

            }

            if (gamepad1.x){
                // pushes intake down all the way
                lif.setPosition(0.0);
                rif.setPosition(1.0);
            }

            if(gamepad1.y){
                rext.setPosition(0.6);
                lext.setPosition(0.3);
                sleep(300);
                lif.setPosition(0.11);
                rif.setPosition(0.89);


            }
            if (gamepad1.right_bumper){
                rext.setPosition(0.7);
                lext.setPosition(0.2);
                sleep(300);
                lif.setPosition(0.11);
                rif.setPosition(0.89);

            }

            if (gamepad1.dpad_left){
                speed = 0.85;
            }

            if (gamepad1.dpad_right){
                speed = 1.0;
            }



            rx = gamepad1.right_stick_x;
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y ;
            ry = -gamepad1.right_stick_y;

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            flmotor.setPower((y  + rx + x) / denominator * speed);
            blmotor.setPower((y  - rx + x) / denominator * speed);
            frmotor.setPower((y - rx - x) / denominator * speed);
            brmotor.setPower((y  + rx - x) / denominator * speed);

            //flmotor.setPower((y+x) / denominator * speed);
            //blmotor.setPower((y-x) / denominator * speed);
            //frmotor.setPower((ry-rx) / denominator * speed);
            //brmotor.setPower((ry+rx) / denominator * speed);
            rightslide.setPower(gamepad2.left_stick_y * speed);
            leftslide.setPower(gamepad2.left_stick_y * speed);
            lift_telemtry();

        }
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
    public void lift_telemtry(){
        telemetry.addData("LL", leftslide.getCurrentPosition());
        telemetry.addData("RL", rightslide.getCurrentPosition());
        telemetry.update();
    }




/*
    public void slidePID(int target){

        controller.setPID(p,i,d);
        controller1.setPID(p1,i1,d1);
        int armPos = leftslide.getCurrentPosition();
        int armPos1 = rightslide.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double pid1 = controller1.calculate(armPos1, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double ff1 = Math.cos(Math.toRadians(target /ticks_in_degree)) * f1;
        double power = pid + ff;
        double power1 = pid1 + ff1;

        leftslide.setPower(power);
        rightslide.setPower(power1);

        telemetry.addData("pos ", armPos);
        telemetry.addData("pos1 ", armPos1);
        telemetry.addData("target ",target);
        telemetry.update();

    }
    cases, need to pushed up the code if being used, needs to be in the while(op mode active) block
    switch (pos){
                case 1:
                    slidePID(100);
                    break;
                case 2:
                    slidePID(0);
                    break;
                case 3:
                    slidePID(1000);
                    break;
                case 4:
                    slidePID(2000);
                    break;
                case 5:
                    slidePID(3000);
                    break;
            }



*/
}

