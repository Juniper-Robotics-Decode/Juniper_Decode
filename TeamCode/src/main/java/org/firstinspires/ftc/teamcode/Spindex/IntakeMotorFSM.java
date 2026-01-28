package org.firstinspires.ftc.teamcode.Spindex;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

public class IntakeMotorFSM {
    public enum states{
        intaking,
        ejecting,
        off,
    }
    public MotorWrapper IntakeMotor;
    public boolean intake; //Detected motif full or not
    public boolean FORCE_STOP=false; //Gamepad button to stop intake!
    public states state;
    private Telemetry telemetry;

    public IntakeMotorFSM(HWMapSpindex hwMap, Telemetry telemetry){
        IntakeMotor = new MotorWrapper(hwMap.getIntakeMotor(),false,1, 537.7);
        this.state = states.intaking;
        this.telemetry = telemetry;
        state= states.intaking; // default
    }
    public void updateState(){
        if (FORCE_STOP) {
            state = states.off;
            IntakeMotor.set(0);
        }
        else if (intake) {
            // intake = true means Spindex has room (not full)
            state = states.intaking;
            IntakeMotor.set(1.0);
        }
        else {
            // intake = false means Spindex is FULL
            state = states.ejecting;
            IntakeMotor.set(-0.5);
            IntakeMotor.brakeModeMotor();
        }
        telemetry.addData("INTAKE STATE?", state);
        telemetry.addData("Intake?", intake);
        telemetry.addData("FORCE STOP?", FORCE_STOP);
        telemetry.update();
    }
}
