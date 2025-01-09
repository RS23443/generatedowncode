package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@Config
@TeleOp
public class IntakeDiffy extends LinearOpMode {
    public static int servorpm = 230; // Servo RPM for calculations
    public static double power = 0.3; // Power level to apply to servos
    public static long targetRotationTime = 1000 * 60 / servorpm; // Time for one full rotation (in ms)
    public static long targetSpinTime = 750 * 60 /servorpm;

    private long starttime = 0;
    private boolean isRunning = false; // Tracks whether servos are running
    private int action = 0; // Tracks the current action: 0 = none, 1 = A, 2 = B, 3 = X, 4 = Y

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo left_intake_diffy = hardwareMap.get(CRServo.class, "left_intake_diffy");
        CRServo right_intake_diffy = hardwareMap.get(CRServo.class, "right_intake_diffy");

        waitForStart();

        while (opModeIsActive()) {
            // Handle button presses
            if (gamepad1.a && !isRunning) {
                startAction(1, left_intake_diffy, right_intake_diffy, power, power);
            } else if (gamepad1.b && !isRunning) {
                startAction(2, left_intake_diffy, right_intake_diffy, -power - 0.1, power + 0.1);
            } else if (gamepad1.x && !isRunning) {
                startAction(3, left_intake_diffy, right_intake_diffy, power, -power);
            } else if (gamepad1.y && !isRunning) {
                startAction(4, left_intake_diffy, right_intake_diffy, -power, -power);
            }

            // Stop servos after the rotation time
            if ((1 < action ||action < 4) && isRunning && (System.currentTimeMillis() - starttime > targetRotationTime)) {
                left_intake_diffy.setPower(0.0);
                right_intake_diffy.setPower(0.0);
                isRunning = false; // Reset the running state
                action = 0; // Clear the current action
            } else if (isRunning){
                if(System.currentTimeMillis() - starttime > targetSpinTime){
                    left_intake_diffy.setPower(0.0);
                    right_intake_diffy.setPower(0.0);
                    isRunning = false; // Reset the running state
                    action = 0; // Clear the current action
                }
            }

            telemetry.addData("Action", action);
            telemetry.addData("Left Servo Power", left_intake_diffy.getPower());
            telemetry.addData("Right Servo Power", right_intake_diffy.getPower());
            telemetry.addData("Elapsed Time (ms)", System.currentTimeMillis() - starttime);
            telemetry.update();
        }
    }

    private void startAction(int actionId, CRServo leftServo, CRServo rightServo, double leftPower, double rightPower) {
        // Start a new action
        starttime = System.currentTimeMillis();
        leftServo.setPower(leftPower);
        rightServo.setPower(rightPower);
        isRunning = true;
        action = actionId;
    }

}
