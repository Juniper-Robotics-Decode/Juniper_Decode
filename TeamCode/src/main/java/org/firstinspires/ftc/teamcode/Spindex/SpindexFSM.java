package org.firstinspires.ftc.teamcode.Spindex;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;

@TeleOp
public class SpindexFSM {

    public enum states {
        STOPPING_AT_TARGET, // spindex moving to target slot
        STOPPED_AT_TARGET,   // spindex reached target slot
    }

    public enum modes {
        SHOOTING,
        INTAKNG,
    }

    private TouchSensorMotorFSM touchSensorMotorFSM;
    private ColorSensorsFSM colorSensorsFSM;
    private states state;
    public  modes mode;

    private Telemetry telemetry;

    public SpindexFSM(HWMap hwMap, Telemetry telemetry) {
        touchSensorMotorFSM = new TouchSensorMotorFSM(hwMap, telemetry);
        colorSensorsFSM = new ColorSensorsFSM(hwMap, telemetry);
        this.telemetry = telemetry;
        state = states.STOPPING_AT_TARGET;
    }


     public void updateState(){
        touchSensorMotorFSM.spindexOffset(mode);
        touchSensorMotorFSM.updateState();
        colorSensorsFSM.updateState();
        //target angle calculation method and finding target like slot stuff will go here
         if (gamepad1.square || gamepad1.circle || gamepad1.triangle){
             mode = modes.SHOOTING;
         }
        switch (state) {
            case STOPPING_AT_TARGET:
                if (gamepad1.square || gamepad1.circle || gamepad1.triangle){

                }
                break;
        }
    }
    //the actual finding and proccess of methods and stuff go here
// need to add the actual motor movement code here with the offset and stuff

    //telemetry


}
