package org.firstinspires.ftc.teamcode.core;


import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.lights.RGBIndicator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HWMapSpindex {

    //Spindex
    private MotorEx spindexMotor;
    private RevColorSensorV3 colorSensor1;
    private RevColorSensorV3 colorSensor2;
    private RevColorSensorV3 colorSensor3;
    private RevTouchSensor TCS1;
    private AnalogInput AI1;
    private ServoEx rgbIndicator1;
    private ServoEx rgbIndicator2;
    private ServoEx rgbIndicator3;

    public HWMapSpindex(HardwareMap hardwareMap) {

        colorSensor1 = hardwareMap.get(RevColorSensorV3.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");
        colorSensor3 = hardwareMap.get(RevColorSensorV3.class, "colorSensor3");
        TCS1 = hardwareMap.get(RevTouchSensor.class, "TCS1");
        AI1 = hardwareMap.get(AnalogInput.class, "AI1");
        spindexMotor = new MotorEx(hardwareMap, "spindexMotor");
        rgbIndicator1 = hardwareMap.get(ServoEx.class, "RI1");
        rgbIndicator2 = hardwareMap.get(ServoEx.class, "RI2");
        rgbIndicator3 = hardwareMap.get(ServoEx.class, "RI3");
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

    public RevColorSensorV3 getColorSensor3() {
        return colorSensor3;
    }

    public RevTouchSensor getTouchSensor1() {
        return TCS1;
    }

    public AnalogInput getAnalogInput1() {
        return AI1;
    }

    public ServoEx getRgbIndicator1() {
        return rgbIndicator1;
    }

    public ServoEx getRgbIndicator2() {
        return rgbIndicator2;
    }

    public ServoEx getRgbIndicator3() {
        return rgbIndicator3;
    }

}