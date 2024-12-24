package org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class intake_subsystem extends SubsystemBase {
    private final Servo lif;
    private final Servo rif;
    private final Servo rext;
    private final Servo lext;
    private final Servo intake;
    private final Servo intakespin;

    // hardwaremap, lifname, rifname, rextname, lextname, intakeclaw name, intakespin name
    public intake_subsystem(final HardwareMap hardwareMap, final String lifname, final String rifname, final String lextname, final String rextname, final String intakeclawname, final String intakespinname) {
        lif = hardwareMap.get(Servo.class, lifname);
        rif = hardwareMap.get(Servo.class, rifname);
        lext = hardwareMap.get(Servo.class, lextname);
        rext = hardwareMap.get(Servo.class, rextname);
        intake = hardwareMap.get(Servo.class, intakeclawname);
        intakespin = hardwareMap.get(Servo.class, intakespinname);
    }

    public void intakeopen(){
        intake.setPosition(0.3);
    }

    public void transferpose() throws InterruptedException{
        intake.setPosition(0.0);
        Thread.sleep(200);
        intakespin.setPosition(0.1);
        Thread.sleep(200);
        lif.setPosition(1.0);
        rif.setPosition(0.0);
        Thread.sleep(200);
        rext.setPosition(0.5);
        lext.setPosition(0.4);
    }

    public void maxextension() throws InterruptedException{
        // can actually be 0.9 and 0.0, but for autonomous sake we didnt, unless we change it on saturday
        intake.setPosition(0.3);
        intakespin.setPosition(0.1);
        rext.setPosition(0.8);
        lext.setPosition(0.10);
        Thread.sleep(300);
        lif.setPosition(0.0);
        rif.setPosition(1.0);

    }

    public void max_ext_for_3rd() throws InterruptedException{
        intake.setPosition(0.3);
        intakespin.setPosition(0.6);
        Thread.sleep(100);
        lif.setPosition(0.3);
        rif.setPosition(0.7);
        Thread.sleep(100);
        rext.setPosition(0.9);
        lext.setPosition(0.0);
        Thread.sleep(300);
        lif.setPosition(0);
        rif.setPosition(1);
    }

    public void intakeclose(){
        intake.setPosition(0.0);
    }

    public void intakespinvertical() throws InterruptedException {
        intakespin.setPosition(0.6);
        Thread.sleep(100);
        lif.setPosition(0.3);
        rif.setPosition(0.7);
    }

    public void intakespinhorizontal(){
        intakespin.setPosition(0.1);
    }

    public void prefireintake() throws InterruptedException{
        intake.setPosition(0.3);
        intakespin.setPosition(0.1);
        rext.setPosition(0.8);
        lext.setPosition(0.10);
        Thread.sleep(300);
        lif.setPosition(0.3);
        rif.setPosition(0.7);

    }

    public void dropdownintake(){
        lif.setPosition(0.15);
        rif.setPosition(0.85);
    }
}

