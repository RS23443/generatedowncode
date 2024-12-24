package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Centric Control", group = "TeleOp")
public class FieldCentric extends LinearOpMode {

    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors and IMU
        initializeHardware();

        telemetry.addLine("Initialization Complete. Press Start to Run.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Get joystick inputs
            double y = -gamepad1.left_stick_y; // Forward/backward (Y is inverted)
            double x = gamepad1.left_stick_x; // Strafing
            double rx = gamepad1.right_stick_x; // Rotation

            // Reset IMU yaw when 'options' button is pressed
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // Calculate robot's heading
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Apply field-centric transformations
            double[] fieldCentricPower = calculateFieldCentricPower(x, y, rx, botHeading);

            // Set motor powers
            setMotorPowers(fieldCentricPower);
        }
    }

    private void initializeHardware() {
        // Map motors to configuration names
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "left_back_drive");

        // Reverse direction for specific motors
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Map IMU to configuration name and set orientation
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
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
        frontLeftMotor.setPower(powers[0]);
        backLeftMotor.setPower(powers[1]);
        frontRightMotor.setPower(powers[2]);
        backRightMotor.setPower(powers[3]);

        // Display telemetry for debugging
        telemetry.addData("Front Left Power", powers[0]);
        telemetry.addData("Back Left Power", powers[1]);
        telemetry.addData("Front Right Power", powers[2]);
        telemetry.addData("Back Right Power", powers[3]);
        telemetry.update();
    }
}
