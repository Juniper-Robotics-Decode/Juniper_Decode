package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

public class IntakeMotorFSm {
    public enum states{
        Intaking,
        Ejecting,
        OFF,
    }
    public MotorWrapper IntakeMotor;
    public int counter;
    public boolean intake; //Detected motif full or not
    public boolean FORCE_STOP;
    public int Counter;
    public states state;
    private Telemetry telemetry;

    public IntakeMotorFSm(HWMapSpindex hwMap, Telemetry telemetry){
        IntakeMotor = new MotorWrapper(hwMap.getSpindexMotor(),false,1, 537.7);
        this.state = states.Intaking;
        this.telemetry = telemetry;
        state= states.Intaking; // default
    }
    public void updateState(){
        if (intake) { //Detected Motif not full
            state = states.Intaking;
        } else { //Detected Motif full
            state = states.Ejecting;
        }
        if (FORCE_STOP && counter == 0) { //Gamepad intake button
            state = states.OFF;
            counter++;
        }
        if (!FORCE_STOP && counter == 1){
            state = states.Intaking;
            counter = 0;
        }
        telemetry.addData("Counter", counter);
        telemetry.addData("IntakeMotorState", state);
        telemetry.addData("Intake?", intake);

    }
}
