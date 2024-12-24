package org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class lifts_subsystem extends SubsystemBase {

    private DcMotorEx rightslides;
    private DcMotorEx leftslides;
    public static double p = 0.019, i = 0.0023, d = 0.000;
    public static double p1 = 0.019, i1 = 0.0023, d1 = 0.000;
    public static double maxticks = 3950.00;
    private double f = 0.05;
    private double f1 = 0.05;
    private final double ticks_in_degree = 537.7 / 180.0;



    public lifts_subsystem(final HardwareMap hardwareMap, final String leftslidename, final String rightslidename) {
        leftslides = hardwareMap.get(DcMotorEx.class, leftslidename);
        rightslides = hardwareMap.get(DcMotorEx.class, rightslidename);


        leftslides.setDirection(DcMotorEx.Direction.REVERSE);
        //leftslides.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        //rightslides.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        //leftslides.setTargetPosition(0);
        //rightslides.setTargetPosition(0);

        //leftslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightslides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
    public void joystick(Gamepad gamepad, int speed){
        rightslides.setPower(gamepad.left_stick_y * speed);
        leftslides.setPower(gamepad.left_stick_y * speed);
    }
    public void slidePID(int target){

        PIDController controllerleft = new PIDController(p,i,d);
        PIDController controllerright= new PIDController(p1,i1,d1);

        int armPos = leftslides.getCurrentPosition();
        int armPos1 = rightslides.getCurrentPosition();
        double pid = controllerleft.calculate(armPos, Math.min(target,maxticks));
        double pid1 = controllerright.calculate(armPos1, Math.min(target,maxticks));
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double ff1 = Math.cos(Math.toRadians(target /ticks_in_degree)) * f1;
        double power = pid + ff;
        double power1 = pid1 + ff1;

        leftslides.setPower(power);
        rightslides.setPower(power1);

    }

    public int getLeftSlidePosition(){
        return leftslides.getCurrentPosition();
    }

    public int getRightSlidePosition() {
        return rightslides.getCurrentPosition();
    }

    public void stopSlides(){
        leftslides.setPower(0.0);
        rightslides.setPower(0.0);
    }

    public void stopandreset(){
        leftslides.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        rightslides.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
    }

    public void runslides(double power){
        rightslides.setPower(power);
        leftslides.setPower(power);
    }

    public void liftrunencoders(){
        leftslides.setMode((DcMotor.RunMode.RUN_USING_ENCODER));
        rightslides.setMode((DcMotor.RunMode.RUN_USING_ENCODER));

    }

    public class Lift implements Action {
        private double power = 0;
        private double ticksToMove = 0;
        private long endTime = 0;

        public Lift(double power, double inches, long secondsToWait) {
            double ticksPerInch = ((4498+4345)/2)/44.5; // need to change this value when the robot is built

            this.power = power;
            this.ticksToMove = (int) (inches * ticksPerInch);

            leftslides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightslides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            leftslides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightslides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            this.endTime = System.currentTimeMillis() + secondsToWait * 1000;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (System.currentTimeMillis() < endTime)
            {
                return true;
            }

            if (Math.abs(getLeftSlidePosition()) < ticksToMove && Math.abs(getRightSlidePosition()) < ticksToMove) {
                rightslides.setPower(power);
                leftslides.setPower(power);
                return true;
            }
            rightslides.setPower(0);
            leftslides.setPower(0);
            return false;
        }
    }

    public Action liftUp(double power, double inches, long secondsToWait) {
        return new Lift(power, inches, secondsToWait);
    }

    public Action liftDown(double power, double inches, long secondsToWait) {
        return new Lift(-power, inches, secondsToWait);
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

            boolean isAtTarget = // (currentLeft > targetPosition || currentRight > targetPosition)
                    (
                            Math.abs(currentLeft - targetPosition) < 75 ||
                                    Math.abs(currentRight - targetPosition) < 75);

            if (isAtTarget) {
                liftSubsystem.stopSlides();
                return false; // Action is complete
            }

            return true; // Action should continue
        }
    }
    public Action moveLiftAction(int targetPosition, lifts_subsystem lifts_subsystem) {
        return new MoveLiftToPosition(lifts_subsystem, targetPosition);
    }
}

