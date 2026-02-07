package org.firstinspires.ftc.teamcode.core;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
 Holds hard ware port names. this has to match the robot config.
 */
public class HWMapTest {

    private final MotorEx intakeMotor;
    private final Servo intakeServo;

    // Transfer
    private final MotorEx transferMotor;
    private final Servo transferServo;


    public HWMapTest (HardwareMap hardwareMap) {

        intakeMotor = new MotorEx(hardwareMap, "IM", Motor.GoBILDA.RPM_1150);
        transferMotor = new MotorEx(hardwareMap, "TRM", Motor.GoBILDA.RPM_312);
        transferServo = hardwareMap.get(Servo.class, "TS");
        intakeServo = hardwareMap.get(Servo.class, "IS");

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

    public Servo getIntakeServo() {
        return intakeServo;
    }


}
