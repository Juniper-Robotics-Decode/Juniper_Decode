package org.firstinspires.ftc.teamcode.Spindex;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;
import org.firstinspires.ftc.teamcode.Spindex.SpindexFSM.modes;
@TeleOp
public class TouchSensorMotorFSM {
    public enum states {
        POSITION_1,
        POSITION_2,
        POSITION_3,
    }

    public static AnalogInput AI1;
    public final MotorWrapper spindexMotor;
    public static states state;
    private Telemetry telemetry;
    int currentIndex;
    double voltage = 4.000;
    public int offset = 0;


    public TouchSensorMotorFSM(HWMapSpindex hwMap, Telemetry telemetry) {
        spindexMotor = new MotorWrapper(hwMap.getSpindexMotor(),false,1, 537.7);

        AI1 = hwMap.getAnalogInput1();

        this.telemetry = telemetry;

        state = states.POSITION_1; // placeholder
    }
    public void spindexOffset(modes mode){
        if(mode == modes.SHOOTING){
            offset = 60;
        }
        if(mode == modes.INTAKNG){
            offset = 0;
        }
    }

    public void updateState() {
        spindexMotor.readPosition();
        double a = Math.abs(spindexMotor.getAngle());
        double relativePositionInRotation = a % 360;

        if (relativePositionInRotation < 120) {
            currentIndex = 1;
        } else if (relativePositionInRotation < 240 && relativePositionInRotation > 120) {
            currentIndex = 2;
        } else {
            currentIndex = 3;
        }

        if (currentIndex == 1) {
            state = states.POSITION_1;
        } else if (currentIndex == 2) {
            state = states.POSITION_2;
        } else if (currentIndex == 3) {
            state = states.POSITION_3;
        }

        telemetry.addData("Spindex State", state);
    }

//    public void spindexIntakingReset_Movement() {
//        //bassically this would reset to intaking mode so it would move in intaking thirds
//        //then it will move 1/3 then stop for 1 it second to let ball in then move, it will keep
//        // going on and on and on
//        while (!AI1.getVoltage() < ) {
//            spindexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            spindexMotor.setPower(1);
//        }
//        spindexMotor.setPower(0);
//        spindexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        spindexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        int l=1;
//        while (!gamepad1.circle || !gamepad1.triangle || !gamepad1.square) {
//            while (l>0) {
//                spindexMotor.setTargetPosition(intakingRotation * l);
//                spindexMotor.setPower(1);
//                spindexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                sleep(3000);
//            }
//        }
//    }


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
