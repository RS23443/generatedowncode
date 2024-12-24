package org.firstinspires.ftc.teamcode.ModulesToImport.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servoController {
    // Array to hold servo references
    private Servo[] servos = new Servo[10];

    // Initialize all servos
    public void initl(HardwareMap hardwareMap, String[] servoNames) {
        if (servoNames.length > 12) {
            throw new IllegalArgumentException("Cannot initialize more than 10 servos.");
        }
        for (int i = 0; i < servoNames.length; i++) {
            servos[i] = hardwareMap.get(Servo.class, servoNames[i]);
        }
    }

    // Set the position of a specific servo
    public void setServoPosition(int servoIndex, double position) {
        if (servoIndex < 0 || servoIndex >= servos.length || servos[servoIndex] == null) {
            throw new IllegalArgumentException("Invalid servo index.");
        }
        if (position < 0.0 || position > 1.0) {
            throw new IllegalArgumentException("Servo position must be between 0.0 and 1.0.");
        }
        servos[servoIndex].setPosition(position);
    }

    // Get the current position of a specific servo
    public double getServoPosition(int servoIndex) {
        if (servoIndex < 0 || servoIndex >= servos.length || servos[servoIndex] == null) {
            throw new IllegalArgumentException("Invalid servo index.");
        }
        return servos[servoIndex].getPosition();
    }

    // Reset all servos to a default position (e.g., 0.5)
    public void resetAllServos(double defaultPosition) {
        for (Servo servo : servos) {
            if (servo != null) {
                servo.setPosition(defaultPosition);
            }
        }
    }
}


/*
Example Usage

@TeleOp(name = "ServoControllerTest", group = "Test")
public class ServoControllerTest extends LinearOpMode {
    private ServoController servoController = new ServoController();

    @Override
    public void runOpMode() {
        // Servo names as defined in the FTC Robot Configuration
        String[] servoNames = {"servo1", "servo2", "servo3", "servo4", "servo5",
                               "servo6", "servo7", "servo8", "servo9", "servo10"};

        // Initialize the ServoController
        telemetry.addLine("Initializing servos...");
        telemetry.update();
        servoController.init(hardwareMap, servoNames);
        telemetry.addLine("Servos initialized!");
        telemetry.update();

        waitForStart();

        // Example: Control servos during the TeleOp
        while (opModeIsActive()) {
            // Move servo1 to position 0.2
            servoController.setServoPosition(0, 0.2);

            // Move servo2 to position 0.8
            servoController.setServoPosition(1, 0.8);

            // Reset all servos to default position (e.g., 0.5)
            if (gamepad1.a) {
                servoController.resetAllServos(0.5);
                telemetry.addLine("All servos reset to 0.5.");
            }

            // Log servo positions
            for (int i = 0; i < 10; i++) {
                try {
                    telemetry.addData("Servo" + (i + 1) + " Position", servoController.getServoPosition(i));
                } catch (IllegalArgumentException e) {
                    telemetry.addData("Servo" + (i + 1), "Not Initialized");
                }
            }
            telemetry.update();
        }
    }
}



 */