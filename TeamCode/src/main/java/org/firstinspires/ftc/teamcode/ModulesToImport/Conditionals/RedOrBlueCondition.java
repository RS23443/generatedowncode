package org.firstinspires.ftc.teamcode.ModulesToImport.Conditionals;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.ModulesToImport.GeneratedCode.ColorBasedActionWrapper;

// Example of a color condition to stop on blue or red
public class RedOrBlueCondition implements ColorBasedActionWrapper.ColorCondition {
    @Override
    public boolean shouldStop(ColorSensor leftSensor, ColorSensor rightSensor, String blueOrRed) {
        if(blueOrRed.toLowerCase().equals("blue")){
            return (leftSensor.blue() > 100 || rightSensor.blue() > 100);
        } else{
            return(rightSensor.red() > 100 || leftSensor.red() > 100);
        }
    }
}
