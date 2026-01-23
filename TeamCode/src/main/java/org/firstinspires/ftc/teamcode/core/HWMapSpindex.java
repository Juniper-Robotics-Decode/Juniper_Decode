package org.firstinspires.ftc.teamcode.core;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HWMapSpindex {

    //Spindex
    private MotorEx spindexMotor;
    private RevColorSensorV3 colorSensor1;
    private RevColorSensorV3 colorSensor2;
    private AnalogInput AI1;

    public HWMapSpindex(HardwareMap hardwareMap) {

        colorSensor1 = hardwareMap.get(RevColorSensorV3.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");
        AI1 = hardwareMap.get(AnalogInput.class, "AI1");
        spindexMotor = new MotorEx(hardwareMap,"spindexMotor", Motor.GoBILDA.RPM_312);
    }


    public MotorEx getSpindexMotor() {
        return spindexMotor;
    }

    public RevColorSensorV3 getColorSensor1() {
        return colorSensor1;
    }

    public RevColorSensorV3 getColorSensor2() {
        return colorSensor2;
    }

    public AnalogInput getAnalogInput1(){
        return AI1;
    }

}
