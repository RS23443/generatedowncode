package org.firstinspires.ftc.teamcode.ModulesToImport.systems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private CRServo intake;
    private Servo lif;
    private Servo lext;
    private Servo rif;
    private Servo rext;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(CRServo.class, "intake");
        lif = hardwareMap.get(Servo.class, "left_intake_flip");
        lext = hardwareMap.get(Servo.class, "left_extension");
        rif = hardwareMap.get(Servo.class, "right_intake_flip");
        rext = hardwareMap.get(Servo.class, "right_extension");

    }
}