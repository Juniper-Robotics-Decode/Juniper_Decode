package org.firstinspires.ftc.teamcode.Spindex;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;

public class SpindexFSM {

    public enum states {
        STOPPING_AT_TARGET, // spindex moving to target slot
        STOPPED_AT_TARGET,   // spindex reached target slot
        IDLING,             //Spining and just collecting/DEFAULT
    }

    private TouchSensorsMotorFSM touchSensorsMotorFSM;
    private ColorSensorsFSM colorSensorsFSM;
    private states state;

    private Telemetry telemetry;
    public SpindexFSM(HWMap hwMap, Telemetry telemetry){
        touchSensorsMotorFSM = new TouchSensorsMotorFSM(hwMap, telemetry);
        colorSensorsFSM = new ColorSensorsFSM(hwMap, telemetry);
        this.telemetry=telemetry;
        state = states.IDLING;
    }
    public void updateState() { //condition here
        touchSensorsMotorFSM.updateState();
        colorSensorsFSM.updateState();
        //target angle calculation method and finding target like slot stuff will go here

        //case, switches and and breaks go here
    }
    //the actual finding and proccess of methods and stuff go here

    //telemetry
}
