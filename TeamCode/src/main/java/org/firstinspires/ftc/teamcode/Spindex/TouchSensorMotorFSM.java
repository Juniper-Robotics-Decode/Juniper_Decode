package org.firstinspires.ftc.teamcode.Spindex;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;
import org.firstinspires.ftc.teamcode.Spindex.SpindexFSM.modes;
@TeleOp
public class TouchSensorMotorFSM {
    public enum states {
        POSITION_1,
        POSITION_2,
        POSITION_3,
    }

    public static RevTouchSensor TCS1;
    public final MotorWrapper spindexMotor;
    public static states state;
    private Telemetry telemetry;
    int currentIndex;


    public TouchSensorMotorFSM(HWMap hwMap, Telemetry telemetry) {
        spindexMotor = new MotorWrapper(hwMap.getSpindexMotor(),false,1, 537.7);

        TCS1 = hwMap.getTouchSensor1();

        double a = Math.abs(spindexMotor.getAngle());

        double relativePositionInRotation = a % 360;

        if (relativePositionInRotation < 120) {
            currentIndex = 1;
        } else if (relativePositionInRotation < 240 && relativePositionInRotation>120) {
            currentIndex = 2;
        } else {
            currentIndex = 3;
        }

        this.telemetry = telemetry;

        state = states.POSITION_1; // placeholder
    }
    public void spindexOffset(modes mode){
        if(mode == modes.SHOOTING){
            int offset = 60;
        }
        if(mode == modes.INTAKNG){
            int offset = 0;
        }
    }

    public void updateState() {
         spindexMotor.readPosition();
        if (currentIndex == 1) {
            state = states.POSITION_1;
        } else if (currentIndex == 2) {
            state = states.POSITION_2;
        } else if (currentIndex == 3) {
            state = states.POSITION_3;
        }

        telemetry.addData("Spindex State", state);
    }

    public boolean atPosition1() {
        return state == states.POSITION_1;
    }

    public boolean atPosition2() {
        return state == states.POSITION_2;
    }

    public boolean atPosition3() {
        return state == states.POSITION_3;
    }
}
