package org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class hang_subsystem extends SubsystemBase {
    private DcMotorEx right_hang;
    private DcMotorEx left_hang;

    public hang_subsystem(final HardwareMap hardwareMap, final String righthangname, final String lefthangname){
        right_hang = hardwareMap.get(DcMotorEx.class,righthangname);
        left_hang = hardwareMap.get(DcMotorEx.class,lefthangname);

        //might have to reverse one dont know yet which one tho
        right_hang.setDirection(DcMotorSimple.Direction.REVERSE);
        right_hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void joysticks(Gamepad gamepad, double speed){
        right_hang.setPower(-gamepad.right_stick_y * speed);
        left_hang.setPower(-gamepad.right_stick_y * speed);

    }

    public void upTriggerBased(Gamepad gamepad, double speed){
        right_hang.setPower(gamepad.right_trigger * Math.abs(speed));
        left_hang.setPower(gamepad.right_trigger * Math.abs(speed));
    }

    public void DownTriggerBased(Gamepad gamepad, double speed){
        right_hang.setPower(gamepad.left_trigger * -Math.abs(speed));
        left_hang.setPower(gamepad.left_trigger * -Math.abs(speed));
    }
}
