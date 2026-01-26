package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

public class IntakeMotorFSm {
    public enum states{
        INTAKING,
        EJECTING,
        OFF,
    }
    public MotorWrapper IntakeMotor;
    public boolean intake; //Detected motif full or not
    public boolean FORCE_STOP; //Gamepad button to stop intake!
    public states state;
    private Telemetry telemetry;

    public IntakeMotorFSm(HWMapSpindex hwMap, Telemetry telemetry){
        IntakeMotor = new MotorWrapper(hwMap.getSpindexMotor(),false,1, 537.7);
        this.state = states.INTAKING;
        this.telemetry = telemetry;
        state= states.INTAKING; // default
    }
    public void updateState(){
        if (intake && !FORCE_STOP) { //Detected Motif not full
            state = states.INTAKING;
        } else if(!intake && !FORCE_STOP){ //Detected Motif full
            state = states.EJECTING;
        } else if (FORCE_STOP) { //stop requested
            state = states.OFF;
//1
        }
        telemetry.addData("INTAKE STATE?", state);
        telemetry.addData("Intake?", intake);
        telemetry.addData("FORCE STOP?", FORCE_STOP);
        telemetry.update();
    }
}
