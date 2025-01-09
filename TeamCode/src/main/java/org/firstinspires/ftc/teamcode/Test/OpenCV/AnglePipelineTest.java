package org.firstinspires.ftc.teamcode.Test.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ModulesToImport.Conditionals.AngleBasedPipeline;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.intake_subsystem;
import org.openftc.easyopencv.OpenCvCamera;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Claw Angle Adjust Test", group = "Test")


public class AnglePipelineTest extends LinearOpMode {
    public int DEGREESPERROTATION = 180;
    public double POSITIONPERDEGREES = 1/DEGREESPERROTATION;
    public double POSITIONDESIRED;
    public intake_subsystem intake;
    public AngleBasedPipeline pipeline;
    public OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        initCamera();
        waitForStart();
        while (opModeIsActive()){
            //intake = new intake_subsystem(hardwareMap, "","","","","","");
            while(opModeIsActive()) {
                double BLOCKANGLE = pipeline.getBlockAngle();
                String BLOCKCOLOR = pipeline.getDominantColor();

                telemetry.addData("Block Angle", BLOCKANGLE);
                telemetry.addData("Block Color", BLOCKCOLOR);
                telemetry.update();

                POSITIONDESIRED = BLOCKANGLE * POSITIONPERDEGREES;

                //intake.spin(POSITIONDESIRED);

            }
            }
    }
    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new AngleBasedPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });
    }
}
