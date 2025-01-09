package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name ="Servo Position Tuner")
public class ServoTuning extends LinearOpMode{

    // intake variables, tune from here
    public static double right_intake_diffy_value = 0.75;
    public static double left_intake_diffy_value = 1 - right_intake_diffy_value;
    public static double intake_claw_value = 0.15;
    public static double right_intake_flip_value = 0.35;
    public static double left_intake_flip_value = 1 - right_intake_flip_value;
    public static double right_intake_extension_value = 0.0;
    public static double left_intake_extension_value = 1 - right_intake_extension_value;
    public static double intake_diffy_spin_multiplier = 0.0;


    // outtake variables tune from here
    public static double right_outtake_diffy_value = 0.75;
    public static double left_outtake_diffy_value = 0.75;
    public static double right_outtake_flip_value = 0.0;
    public static double left_outtake_flip_value = 1 - right_outtake_flip_value;
    public static double outtake_claw_value = 0.35;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Gamepad A is for Intake");
        telemetry.addLine("Gamepad B is for Outtake");

        // initialize the servos here
        // intake servos
        // Control Hub, order 0 to 3/top-down
        Servo left_intake_extension = hardwareMap.get(Servo.class, "left_intake_ext");
        Servo left_intake_flip = hardwareMap.get(Servo.class, "left_intake_flip");
        Servo left_intake_diffy = hardwareMap.get(Servo.class, "left_intake_diffy");
        Servo intake_claw = hardwareMap.get(Servo.class, "intake_claw");

        // Expansion Hub, order 3 to 5/ top-down
        Servo right_intake_diffy = hardwareMap.get(Servo.class, "right_intake_diffy");
        Servo right_intake_flip = hardwareMap.get(Servo.class, "right_intake_flip");
        Servo right_intake_extension = hardwareMap.get(Servo.class, "right_intake_ext");

        // outtake servos
        // Control Hub, order 4 to 5/ top-down
        Servo left_outtake_flip = hardwareMap.get(Servo.class, "left_outtake_flip");
        Servo left_outtake_diffy = hardwareMap.get(Servo.class, "left_outtake_diffy");
        // Expansion Hub, order 0 to 2/ top-down
        Servo outttake_claw = hardwareMap.get(Servo.class, "outtake_claw");
        Servo right_outtake_diffy = hardwareMap.get(Servo.class, "right_outtake_diffy");
        Servo right_outake_flip = hardwareMap.get(Servo.class, "right_outtake_flip");
        waitForStart();
        while (opModeIsActive()) {

            //initialize and run all booleans
            isRightTriggerPressed_Gamepad1();
            isLeftTriggerPressed_Gamepad1();

            //Gamepad A - Intake Controls
            // spins the diffy
            if (gamepad1.a) {
                left_intake_diffy.setPosition(0.6); // starts at the int value
                right_intake_diffy.setPosition(0.4); // starts at the init value
            }
            // diffy down 4 bar need to change val hark stuck
            if (gamepad1.b) { // moves down towards the intake
               left_intake_diffy.setPosition(0.1); // towards 1
                right_intake_diffy.setPosition(0.9); // towards 0
                // should move the diffy up and down
            }

            //diffy up 4 bar
            if (gamepad1.x) {
                //towards the same number
                left_intake_diffy.setPosition(1.0); // towards 0
                right_intake_diffy.setPosition(0.8); // towards 0
            }

            if (gamepad1.y) {
                left_intake_diffy.setPosition(0.2); // 1.0
                right_intake_diffy.setPosition(0.0); //1.0
            }
            // intake extension
            if (gamepad1.right_bumper) { // -> retraction
                right_intake_extension.setPosition(right_intake_extension_value); // 0.0
                left_intake_extension.setPosition(left_intake_extension_value); // 1.0
            }

            if (gamepad1.left_bumper) { // -> extension
                right_intake_extension.setPosition(1 - right_intake_extension_value); // 0.2
                left_intake_extension.setPosition(1 - left_intake_extension_value); // 0.8
            }
            // intake flip
            if (gamepad1.dpad_right) { // -> flip in
                left_intake_flip.setPosition(left_intake_flip_value); // 0.65
                right_intake_flip.setPosition(right_intake_flip_value); // 0.35
            }

            if (gamepad1.dpad_left) { // -> flip out
                left_intake_flip.setPosition(1 - left_intake_flip_value); // 1 - 0.6 = 0.4
                right_intake_flip.setPosition(1 - right_intake_flip_value); // 1 - 0.4 = 0.6
            }
            //intake claw
            if (gamepad1.dpad_up) { // -> open claw
                intake_claw.setPosition(intake_claw_value);
            }

            if (gamepad1.dpad_down) { // close claw
                intake_claw.setPosition(0.55 - intake_claw_value); //-> = 0.4
            }

            // Gamepad B - Outtake Controls
            //forward and backward diffy -> if one goes to zero the other one should go to 1 to flip and if the numbers are the going to the same number they will spin
            if(gamepad2.dpad_right){
                right_outtake_diffy.setPosition(2 * right_outtake_diffy_value); // towards 1
                left_outtake_diffy.setPosition(0 * left_outtake_diffy_value); // towards 0
            }

            if (gamepad2.a) { // good position 0.75, 0.75 for teleop can make 0.75-right and 0.25-left
                right_outtake_diffy.setPosition(right_outtake_diffy_value); // init value
                left_outtake_diffy.setPosition(left_outtake_diffy_value); // init value
            }
            if (gamepad2.b) {
                left_outtake_diffy.setPosition(2 * left_outtake_diffy_value); // towards 1
                right_outtake_diffy.setPosition(0 * right_outtake_diffy_value); // towards 0
                // should move the diffy up and down
            }

            //diffy spin both ways
            if (gamepad2.x) {
                //towards the same number
                left_outtake_diffy.setPosition(left_outtake_diffy_value * 0); // towards 0
                right_outtake_diffy.setPosition(0 * right_outtake_diffy_value); // towards 0
            }

            if (gamepad2.y) {
                // towards same number
                left_outtake_diffy.setPosition(left_outtake_diffy_value + 0.25); // towards 1
                right_outtake_diffy.setPosition(right_outtake_diffy_value + 0.25); // towards 1
            }
            //outtake flip
            if (gamepad2.left_bumper) { // flips putisde
                left_outtake_flip.setPosition(left_outtake_flip_value); // 0.7
                right_outake_flip.setPosition(right_outtake_flip_value); // 0.3
            }

            if (gamepad2.right_bumper) { // flips inside, 1.0 for right and 0.0 for should be reset position
                left_outtake_flip.setPosition(1 - left_outtake_flip_value);
                right_outake_flip.setPosition(1 - right_outtake_flip_value);
            }

            // outtake claw
            if (gamepad2.dpad_up) {
                outttake_claw.setPosition(outtake_claw_value);
            }

            if (gamepad2.dpad_down) {
                outttake_claw.setPosition(0.92 - outtake_claw_value); //-> needs to equal 0.58
            }
        }
    }

    public boolean isRightTriggerPressed_Gamepad1() {
        if(gamepad1.right_trigger > 0.5)
            return true;
        else{
            return false;
        }
    }

    public boolean isLeftTriggerPressed_Gamepad1() {
        if(gamepad1.left_trigger > 0.5)
            return true;
        else{
            return false;
        }
    }
}
