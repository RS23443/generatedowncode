package org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems;
import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class deposit_subsystem extends SubsystemBase {
    private final intake_subsystem clawtake;

    private final Servo lof;
    private final Servo rof;
    private final Servo o_rightdiffy;
    private final Servo o_leftdiffy;
    private final Servo o_claw;

    public deposit_subsystem(final intake_subsystem intakeSubsystem, final HardwareMap hardwareMap, final String lofname, final String rofname, final String ldiffyname, final String rdiffyname, final String oclawname) {
        this.clawtake = intakeSubsystem;
        lof = hardwareMap.get(Servo.class, lofname);
        rof = hardwareMap.get(Servo.class,rofname);
        o_leftdiffy = hardwareMap.get(Servo.class,ldiffyname);
        o_rightdiffy = hardwareMap.get(Servo.class, rdiffyname);
        o_claw = hardwareMap.get(Servo.class,oclawname);
    }

    public void grabpose() throws InterruptedException {
        //min
        open();
        o_rightdiffy.setPosition(0.75);
        o_leftdiffy.setPosition(0.75);
        lof.setPosition(0.0);
        rof.setPosition(1.0);
    }

    public void dropose() throws InterruptedException{
        block_switch();
        Thread.sleep(100);
        lof.setPosition(0.7); // 0.7
        rof.setPosition(0.3); // 0.3
        Thread.sleep(100);
        //teleop diffy pose btw
        o_rightdiffy.setPosition(0.75);
        o_leftdiffy.setPosition(0.25);
    }

    public void block_switch() throws InterruptedException{
        close();
        Thread.sleep(50,50);
        clawtake.intakeopen();
    }

    public void open(){
        o_claw.setPosition(0.35);
    }

    public void close(){
        o_claw.setPosition(0.58);
    }
    public void spintospec(){
        o_leftdiffy.setPosition(0.0);
        o_rightdiffy.setPosition(1.0);
    }

}
