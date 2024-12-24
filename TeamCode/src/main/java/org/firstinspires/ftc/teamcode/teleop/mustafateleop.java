package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.localizers.ThreeDeadWheelLocalizer;

@Config
@TeleOp(name="must_teleop", group="LinearOpMode")
public class mustafateleop extends LinearOpMode {
    // wifi name: 23443-RC, password:RoboSupport23443
    private IMU imu;

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

    public double denominator;
    public double speed = 1.0;

    public boolean righttriggerbool = false;
    public boolean lefttriggerbool = false;

    public int maxTicksForLifts = 3970;


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

        //Imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);



        // localizer
        ThreeDeadWheelLocalizer mylocalizer = new ThreeDeadWheelLocalizer(hardwareMap, 0.00199911150599733451799200355398);
        //for encoders reverse left


        //init pose
        //rof.setPosition(0.65);
        //lof.setPosition(0.35);
        //sleep(200);
        //outext.setPosition(0.5);//find the exact position to flip
        //sleep(500);
        //finger.setPosition(0.33);
        //spin.setPosition(0.0);

        waitForStart();
        while (opModeIsActive()) {
            //getColor();
            //colorTelementry();
            checklefttrigger();
            checkrighttrigger();
            if(righttriggerbool){
                lift_control(1.0,3700);
            }
            if (lefttriggerbool){
                lift_control(-1.0,0);
            }

            if (gamepad1.left_bumper){
                //min
                finger.setPosition(0.0);
                sleep(100);
                rof.setPosition(0.85);
                lof.setPosition(0.15);
                outext.setPosition(0.8);
                sleep(100);
                spin.setPosition(0.00);

            }

            if (gamepad1.right_bumper){
                //max
                // will have to bring the claw down because it starts elevated (d_pad down)
                // will cause the arm to flip out
                finger.setPosition(0.65); // close the finger
                sleep(200);
                intakespin.setPosition(0.3);
                sleep(400);
                rof.setPosition(0.55);
                lof.setPosition(0.45);
                spin.setPosition(0.0);
                sleep(100);
                outext.setPosition(0.0);
                righttriggerbool = false;

            }

            if (gamepad1.b){
                intakespin.setPosition(0.0);
                sleep(100);
                intake.setPosition(0.1);
                sleep(200);
                lif.setPosition(1.0);
                rif.setPosition(0.0);
                sleep(200);
                rext.setPosition(0.5);
                lext.setPosition(0.4);
                sleep(200);
                finger.setPosition(0.65); // close the finger
                sleep(200);
                intakespin.setPosition(0.3);
                sleep(400);
                rof.setPosition(0.55);
                lof.setPosition(0.45);
                spin.setPosition(0.0);
                sleep(100);
                outext.setPosition(0.0);
            }

            if(gamepad1.y){
                finger.setPosition(0.0);
                intakespin.setPosition(0.3);

            }


            if (gamepad1.x){
                //vertical
                intake.setPosition(0.6);
            }


            if (gamepad1.a) {
                // sets extensions to max
                intakespin.setPosition(0.3); // opens claw
                rext.setPosition(0.9);
                lext.setPosition(0.00);
                sleep(300);
                lif.setPosition(0.175);
                rif.setPosition(0.835);

            }
            // Reset IMU yaw when 'options' button is pressed
            if (gamepad1.dpad_up){
                imu.resetYaw();
            }

            if (gamepad1.dpad_left){
                speed = 0.75;
            }

            if (gamepad1.dpad_right){
                speed = 1.0;
            }



            double y = -gamepad1.left_stick_y; // Forward/backward (Y is inverted)
            double x = gamepad1.left_stick_x; // Strafing
            double rx = gamepad1.right_stick_x; // Rotation

            // Calculate robot's heading
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Apply field-centric transformations
            double[] fieldCentricPower = calculateFieldCentricPower(x, y, rx, botHeading);

            // Set motor powers
            setMotorPowers(fieldCentricPower);

            telemetry.addData("LL", leftslide.getCurrentPosition());
            telemetry.addData("RL", rightslide.getCurrentPosition());
            telemetry.update();

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

    public void checkrighttrigger(){
        if(gamepad2.right_trigger >= 0.5){
            righttriggerbool = true;
        }
        else{
            righttriggerbool = false;
        }
    }

    public void checklefttrigger(){
        if(gamepad2.left_trigger >= 0.5){
            lefttriggerbool = true;
        }
        else{
            lefttriggerbool= false;
        }
    }

    public void autoclose(){
        if(redValue >= targetValue){
            intakespin.setPosition(0.0);
        }
    }
        private double[] calculateFieldCentricPower(double x, double y, double rx, double botHeading) {
            // Rotate joystick inputs by robot's heading to achieve field-centric control
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Slight adjustment for imperfect strafing
            rotX *= 1.1;

            // Calculate motor power values
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            return new double[]{frontLeftPower, backLeftPower, frontRightPower, backRightPower};
        }

        private void setMotorPowers(double[] powers) {
            flmotor.setPower(powers[0]);
            blmotor.setPower(powers[1]);
            frmotor.setPower(powers[2]);
            brmotor.setPower(powers[3]);

            // Display telemetry for debugging
            telemetry.addData("Front Left Power", powers[0]);
            telemetry.addData("Back Left Power", powers[1]);
            telemetry.addData("Front Right Power", powers[2]);
            telemetry.addData("Back Right Power", powers[3]);
            telemetry.update();
        }

        private void lift_control(double power, int targetposition) {
            rightslide.setPower(power);
            leftslide.setPower(power);
            int rp = rightslide.getCurrentPosition();
            int lp = leftslide.getCurrentPosition();
            int distancetotargetposition = Math.abs((rp+lp)/2 - targetposition);
            if(distancetotargetposition <= 200) {
                double caluclated_power = power * distancetotargetposition/200;
                rightslide.setPower(caluclated_power);
                leftslide.setPower(caluclated_power);
            }
            }
        }




