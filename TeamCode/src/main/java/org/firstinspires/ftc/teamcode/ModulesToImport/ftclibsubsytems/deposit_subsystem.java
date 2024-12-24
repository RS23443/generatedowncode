package org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems;
import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class deposit_subsystem extends SubsystemBase {
    //private final intake_subsystem clawtake;

    private final Servo lof;
    private final Servo rof;
    private final Servo outext;
    private final Servo spin;
    private final Servo finger;

    public deposit_subsystem(/*intake_subsystem clawtake,*/ final HardwareMap hardwareMap, final String lofname, final String rofname, final String outextname, final String spinname, final String fingername) {
        //this.clawtake = clawtake;
        lof = hardwareMap.get(Servo.class, lofname);
        rof = hardwareMap.get(Servo.class,rofname);
        outext = hardwareMap.get(Servo.class,outextname);
        spin = hardwareMap.get(Servo.class, spinname);
        finger = hardwareMap.get(Servo.class,fingername);
    }

    /**
     * Grabs a stone.
     */
    public void grabpose() throws InterruptedException {
        finger.setPosition(0.0);
        sleep(100);
        rof.setPosition(0.85);
        lof.setPosition(0.15);
        outext.setPosition(0.9);
        sleep(100);
        spin.setPosition(0.00);
    }

    /**
     * Releases a stone.
     */
    public void dropose() throws InterruptedException{
        finger.setPosition(0.65); // close the finger
        rof.setPosition(0.45);
        lof.setPosition(0.55);
        spin.setPosition(0.0);
        sleep(100);
        outext.setPosition(0.3);
        sleep(200);
    }

    public void goinguppose() throws InterruptedException{
        //finger.setPosition(0.67); // close the finger
        //sleep(200);
        //clawtake.intakeopen();
        rof.setPosition(0.6);//0.55
        lof.setPosition(0.4); //p.45
        spin.setPosition(0.0);
        sleep(100);
        outext.setPosition(0.5); //0.0
    }

    public void goingdownpose() throws InterruptedException{
        goinguppose();
        finger.setPosition(0.0);
    }

    public void open(){
        finger.setPosition(0.0);
    }

    public void close(){
        finger.setPosition(0.67);
    }

    public void depo_preload() throws InterruptedException{
        finger.setPosition(0.67);
        goinguppose();
    }

}
