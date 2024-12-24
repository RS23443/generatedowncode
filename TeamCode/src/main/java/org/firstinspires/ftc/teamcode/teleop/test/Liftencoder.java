package org.firstinspires.ftc.teamcode.teleop.test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.lifts_subsystem;

@TeleOp
public class Liftencoder extends LinearOpMode {
    @Override
    public void runOpMode() {
        lifts_subsystem lifts = new lifts_subsystem(hardwareMap, "leftslide", "rightslide");
        lifts.stopandreset();
        waitForStart();

        while (opModeIsActive()) {

            lifts.joystick(gamepad2,1);

            telemetry.addData("LL", lifts.getLeftSlidePosition());
            telemetry.addData("RL", lifts.getRightSlidePosition());
            telemetry.update();
        }
    }
}