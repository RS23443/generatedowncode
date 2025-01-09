package org.firstinspires.ftc.teamcode.Test.ColorSensorImplementation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
@TeleOp(name = "color Calibration for Color Sensor")
public class ColorCalibration_ColorSensor extends LinearOpMode {
    public ColorSensor clawColorSensor;
    public double clawredValue,clawgreenValue, clawalphaValue, clawblueValue;
    public boolean isRed = false;
    public boolean isYelllow = false;
    public boolean isBlue = false;

    @Override
    public void runOpMode() throws InterruptedException {
        clawColorSensor = hardwareMap.get(ColorSensor.class, "clawColorSensor");
        waitForStart();
        while(opModeIsActive()){
            getColor();
            colorTelementry();
            isBlue = clawColorSensor.blue() > 1000;
            isYelllow = clawColorSensor.green() > 1000;
            isRed = clawColorSensor.red() > 1000;
            readcolor();
        }
    }

    private void getColor() {
        clawredValue = clawColorSensor.red();
        clawgreenValue = clawColorSensor.green();
        clawblueValue = clawColorSensor.blue();
        clawalphaValue = clawColorSensor.alpha();
    }
    public void colorTelementry(){
        telemetry.addData("redValue","%.3f", clawredValue);
        telemetry.addData("greenValue","%.3f", clawgreenValue);
        telemetry.addData("blueValue","%.3f", clawblueValue);
        telemetry.addData("alphaValue","%.3f", clawalphaValue);
        telemetry.update();

    }

    public void readcolor(){
        if(isBlue){
            telemetry.addData("Color is Blue",isBlue);
            telemetry.update();
        }

        if(isYelllow){
            telemetry.addData("Color is Yellow",isYelllow);
            telemetry.update();
        }

        if(isRed){
            telemetry.addData("Color is Red",isRed);
            telemetry.update();
        }
    }

}
