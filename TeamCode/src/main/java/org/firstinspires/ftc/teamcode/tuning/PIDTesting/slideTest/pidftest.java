package org.firstinspires.ftc.teamcode.tuning.PIDTesting.slideTest;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Auto.sampleonly;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.lifts_subsystem;
@Disabled
@Config
@Autonomous(name="pidftest", group ="Autonomous")
public class pidftest extends LinearOpMode {
    public class Lifts{
        private lifts_subsystem liftss;
        public Lifts(HardwareMap hardwareMap){
            this.liftss = new lifts_subsystem(hardwareMap, "leftslide","rightslide");

        }

        public class movetoposition implements Action {
            private final lifts_subsystem liftSubsystem;
            private final int targetPosition;

            public movetoposition(lifts_subsystem liftSubsystem, int targetPosition){
                this.liftSubsystem = liftSubsystem;
                this.targetPosition = targetPosition; // Passed as a parameter
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                liftSubsystem.slidePID(targetPosition);

                int currentLeft = liftSubsystem.getLeftSlidePosition();
                int currentRight = liftSubsystem.getRightSlidePosition();
                boolean isAtTarget =
                        (
                                Math.abs(liftSubsystem.getLeftSlidePosition()- targetPosition) < 75 || Math.abs(liftSubsystem.getRightSlidePosition() - targetPosition) < 75);

                if (isAtTarget) {
                    sleep(2000);
                   liftSubsystem.stopSlides();
                    //liftSubsystem.stopSlides();
                    return false; // Action is complete
                }

                return true; // Action should continue
            }
        }
        public Action moveLiftAction(int targetPosition) {
            return new Lifts.movetoposition(liftss, targetPosition);
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Lifts lifts = new Lifts(hardwareMap);
        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        lifts.moveLiftAction(1000)

                )
        );
    }
}
