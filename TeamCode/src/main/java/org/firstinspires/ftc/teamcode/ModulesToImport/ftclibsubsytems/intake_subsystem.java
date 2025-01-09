package org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ModulesToImport.Conditionals.EnhancedPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class intake_subsystem extends SubsystemBase {
    private final Servo lif;
    private final Servo rif;
    private final Servo rext;
    private final Servo lext;
    private final Servo claw;
    private final Servo rightdiffy;
    private final Servo leftdiffy;

    private EnhancedPipeline pipeline;
    private OpenCvCamera webcam;
    private boolean isCameraInitialized = false;

    public intake_subsystem(final HardwareMap hardwareMap, final String lifname, final String rifname,
                            final String lextname, final String rextname,
                            final String rightdiffyname, final String leftdiffyname,
                            final String clawname) {
        lif = hardwareMap.get(Servo.class, lifname);
        rif = hardwareMap.get(Servo.class, rifname);
        lext = hardwareMap.get(Servo.class, lextname);
        rext = hardwareMap.get(Servo.class, rextname);
        claw = hardwareMap.get(Servo.class, clawname);
        leftdiffy = hardwareMap.get(Servo.class, leftdiffyname);
        rightdiffy = hardwareMap.get(Servo.class, rightdiffyname);
    }

    public void initCamera(HardwareMap hardwareMap, Telemetry telemetry) {
        if (isCameraInitialized) {
            telemetry.addLine("Camera already initialized.");
            telemetry.update();
            return; // Prevent reinitialization
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new EnhancedPipeline(); // Assign pipeline here
        webcam.setPipeline(pipeline);

        telemetry.addLine("Initializing camera...");
        telemetry.update();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                isCameraInitialized = true;
                telemetry.addLine("Camera initialized successfully.");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Error code: %d", errorCode);
                telemetry.update();
            }
        });
    }


    public OpenCvCamera getCamera() {
        if (!isCameraInitialized) {
            throw new IllegalStateException("Camera has not been initialized yet!");
        }
        return webcam;
    }

    public EnhancedPipeline getPipeline() {
        if (!isCameraInitialized || pipeline == null) {
            throw new IllegalStateException("Pipeline has not been initialized yet! Ensure initCamera() has been called and completed.");
        }
        return pipeline;
    }


    public boolean isCameraInitialized() {
        return isCameraInitialized;
    }

    // Intake actions
    public void intakeopen() {
        claw.setPosition(0.15);
    }

    public void intakeclose() {
        claw.setPosition(0.4);
    }

    public void transferpose() throws InterruptedException {
        claw.setPosition(0.4);
        Thread.sleep(100);

        leftdiffy.setPosition(0.1);
        rightdiffy.setPosition(0.9);
        Thread.sleep(50);

        bringinintake();
        lext.setPosition(1.0);
        rext.setPosition(0.0);
    }

    public void bringinintake() {
        lif.setPosition(0.65);
        rif.setPosition(0.35);
    }

    public void dropdownintake() {
        lif.setPosition(0.39);
        rif.setPosition(0.61);
    }

    public void prefireintake() throws InterruptedException {
        bringinintake();
        rext.setPosition(0.2);
        lext.setPosition(0.8);
        Thread.sleep(200);
        intakespinvertical();
    }

    public void intakespinvertical() {
        leftdiffy.setPosition(0.6);
        rightdiffy.setPosition(0.4);
    }

    public void intakespinhorizontal() throws InterruptedException {
        leftdiffy.setPosition(0.2);
        rightdiffy.setPosition(0.0);
    }
}
