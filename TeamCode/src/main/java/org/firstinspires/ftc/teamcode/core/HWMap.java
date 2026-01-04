package org.firstinspires.ftc.teamcode.core;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;

public class HWMap {
    // shooter
    private final MotorEx flywheelMotor;
    private final MotorEx turretMotor;
    private final CRServo pitchServo;
    private final AnalogInput pitchEncoder;
    private final Limelight3A limelight;

    //intake
    private final MotorEx intakeMotor;

    // Transfer
    private final MotorEx transferMotor;
    private final Servo transferServo;

    //Spindex
    private MotorEx spindexMotor;
    private RevColorSensorV3 colorSensor1;
    private RevColorSensorV3 colorSensor2;
    private RevColorSensorV3 colorSensor3;
    private RevTouchSensor TCS1;
    private AnalogInput AI1;
    private GoBildaPinpointDriver odo;

    public HWMap (HardwareMap hardwareMap) {
        flywheelMotor = new MotorEx(hardwareMap,"FM", Motor.GoBILDA.BARE);
        flywheelMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        turretMotor = new MotorEx(hardwareMap,"TM", Motor.GoBILDA.RPM_1150); // TODO: get right RPM
        pitchServo = new CRServo(hardwareMap, "PS");
        pitchEncoder = hardwareMap.get(AnalogInput.class, "PE");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        intakeMotor = new MotorEx(hardwareMap, "IM", Motor.GoBILDA.RPM_1150);
        transferMotor = new MotorEx(hardwareMap, "TRM", Motor.GoBILDA.RPM_312);
        transferServo = hardwareMap.get(Servo.class, "TS");
        transferServo.setDirection(Servo.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        colorSensor1 = hardwareMap.get(RevColorSensorV3.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");
        colorSensor3 = hardwareMap.get(RevColorSensorV3.class, "colorSensor3");
        TCS1 = hardwareMap.get(RevTouchSensor.class, "TCS1");
        AI1 = hardwareMap.get(AnalogInput.class, "AI1");
        spindexMotor = new MotorEx(hardwareMap,"spindexMotor");
    }


    public Limelight3A getLimelight() {
        return limelight;
    }

    public MotorEx getFlywheelMotor() {
        return flywheelMotor;
    }

    public MotorEx getTurretMotor() {
        return turretMotor;
    }

    public CRServo getPitchServo() {
        return pitchServo;
    }

    public AnalogInput getPitchEncoder() {
        return pitchEncoder;
    }


    public MotorEx getIntakeMotor() {
        return intakeMotor;
    }

    public MotorEx getTransferMotor() {
        return transferMotor;
    }

    public Servo getTransferServo() {
        return transferServo;
    }

    public GoBildaPinpointDriver getOdo() {
        return odo;
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
    public AnalogInput getAnalogInput1(){
        return AI1;
    }

}
