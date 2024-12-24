package org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.deposit_subsystem;
import org.firstinspires.ftc.teamcode.ModulesToImport.ftclibsubsytems.intake_subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot_Controller extends SubsystemBase {
    private  intake_subsystem intake;
    private  deposit_subsystem depo;
    public Robot_Controller (intake_subsystem intake, deposit_subsystem depo){
        this.intake = intake;
        this.depo = depo;
    }

    public void drophalfwaypose() throws InterruptedException {
        depo.close();
        Thread.sleep(200);
        intake.intakeopen();
        depo.goinguppose();
    }

    public void droppose() throws InterruptedException{
        depo.dropose();
    }

    public void goingdown() throws InterruptedException{
        depo.goingdownpose();
    }
    public void depotransfer() throws InterruptedException{
        depo.grabpose();
    }

    public void depopreload() throws InterruptedException{
        depo.depo_preload();
    }
    public void fingerclose(){
        depo.close();
    }

    public void fingeropen(){
        depo.open();
    }

    public void ext_max() throws InterruptedException{
        intake.maxextension();
    }

    public void ext_max_3rd() throws InterruptedException{
        intake.max_ext_for_3rd();
    }

    public void transfer() throws InterruptedException{
        intake.transferpose();
    }

    public void int_open(){
        intake.intakeopen();
    }

    public void int_close() {
        intake.intakeclose();
    }

    public void int_vert_spin() throws InterruptedException{
        intake.intakespinvertical();
    }

    public void int_horizontal_spin() {
        intake.intakespinhorizontal();
    }
    public void preintake() throws InterruptedException{
        intake.prefireintake();
    }

    public void dropintake(){
        intake.dropdownintake();
    }



}
