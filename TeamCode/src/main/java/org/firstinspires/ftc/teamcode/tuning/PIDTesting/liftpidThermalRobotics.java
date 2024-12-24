package org.firstinspires.ftc.teamcode.tuning.PIDTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@Disabled
@TeleOp(name="thermal pid", group="LinearOpMode")

public class liftpidThermalRobotics extends OpMode {
    DcMotorEx leftslide;
    DcMotorEx rightslide;
    public static double integral_sum = 0;
    public static double Kp = 0.0059;
    public static double Ki = 0;
    public static double Kd = 0;

    private double lastError = 0;


    public static double reference = 1000;

    ElapsedTime timer = new ElapsedTime();


    public boolean pointreached = false;

    @Override
    public void init() {
        leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
        //rightslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftslide.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {
        double state = leftslide.getCurrentPosition();
        double error = reference - state;
        integral_sum = error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        double power = ((Kp * error) + (Ki * integral_sum) + (Kd * derivative));
        leftslide.setPower(power);
        rightslide.setPower(power);
        reference = leftslide.getCurrentPosition();

        boolean pointreached = (state == reference);

        if (pointreached){
            leftslide.setPower(0.0);
            rightslide.setPower(0.0);
        }


        int armPos = leftslide.getCurrentPosition();
        int armPos1 = rightslide.getCurrentPosition();
        telemetry.addData("leftslide ", armPos);
        telemetry.addData("righslide ", armPos1);
        telemetry.addData("target ", reference);
        telemetry.update();
        }


    /*public void slidePIDcontroller(double reference, double state) {
        double error = reference - state;
        integral_sum = error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        while (setPointIsNotReached) {
            double power = ((Kp*error) + (Ki*integral_sum) + (Kd*derivative));
            leftslide.setPower(power);
            rightslide.setPower(power);

            if (reference == state){
                setPointIsNotReached = false;
                break;
            }


        }

    }*/
    }
