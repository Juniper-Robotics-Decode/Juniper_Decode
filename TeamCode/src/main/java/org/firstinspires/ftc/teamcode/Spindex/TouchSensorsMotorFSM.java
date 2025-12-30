package org.firstinspires.ftc.teamcode.Spindex;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.motorWrapperSpindex;

@TeleOp
public class TouchSensorsMotorFSM {
    public enum states {
        POSITION_1,
        POSITION_2,
        POSITION_3,
    }

    public static RevTouchSensor TCS1;
    public static RevTouchSensor TCS2;
    public static RevTouchSensor TCS3;
    public final motorWrapperSpindex spindexMotor;
    public static states state;
    private Telemetry telemetry;


    public TouchSensorsMotorFSM(HWMap hwMap, Telemetry telemetry) {
        spindexMotor = new motorWrapperSpindex(hwMap.getSpindexMotor());

        TCS1 = hwMap.getTouchSensor1();
        TCS2 = hwMap.getTouchSensor2();
        TCS3 = hwMap.getTouchSensor3();

        this.telemetry = telemetry;

        state = states.POSITION_1; // placeholder
    }

    public void updateState() {

        if (TCS1.isPressed()) {
            state = states.POSITION_1;
        } else if (TCS2.isPressed()) {
            state = states.POSITION_2;
        } else if (TCS3.isPressed()) {
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
