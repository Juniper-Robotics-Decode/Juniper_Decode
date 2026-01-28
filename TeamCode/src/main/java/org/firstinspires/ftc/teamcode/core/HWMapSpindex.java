package org.firstinspires.ftc.teamcode.core;


import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HWMapSpindex {

    //Spindex
    private MotorEx spindexMotor;

    private MotorEx IntakeMotor;
    private RevColorSensorV3 colorSensor1;
    private DigitalChannel AI;
    public boolean aiSTATE;


    public HWMapSpindex(HardwareMap hardwareMap) {

        colorSensor1 = hardwareMap.get(RevColorSensorV3.class, "colorSensor1");
        AI = hardwareMap.get(DigitalChannel.class, "AI");
        spindexMotor = new MotorEx(hardwareMap, "spindexMotor");
        IntakeMotor = new MotorEx(hardwareMap, "IntakeMotor");
    }


    public MotorEx getSpindexMotor() {
        return spindexMotor;
    }

    public MotorEx getIntakeMotor(){return IntakeMotor;}

    public RevColorSensorV3 getColorSensor1() {
        return colorSensor1;
    }

    public DigitalChannel getAnalogInput() {
        return AI;
    }

    public boolean getAIstate() {
        return AI.getState();
    }
}
