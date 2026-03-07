package org.firstinspires.ftc.teamcode.core;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.intaketransfer.AxonServoWrapper;

public class TestHardwareMap {
    // shooter

    private final MotorEx intakeMotor;

    // Transfer
    private final MotorEx transferMotor;
    private final Servo transferServo;

    private final Servo intakeServo;

    private final AnalogInput transferEncoder;



    public TestHardwareMap (HardwareMap hardwareMap) {


        intakeMotor = new MotorEx(hardwareMap, "IM", Motor.GoBILDA.RPM_1150);
        transferMotor = new MotorEx(hardwareMap, "TRM", Motor.GoBILDA.RPM_312);
        transferServo = hardwareMap.get(Servo.class, "TS");
        intakeServo = hardwareMap.get(Servo.class, "IS");
        transferEncoder = hardwareMap.get(AnalogInput.class, "TE");
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

    public Servo getIntakeServo(){return intakeServo;}

    public AnalogInput getTransferEncoder() {return transferEncoder;}


}
