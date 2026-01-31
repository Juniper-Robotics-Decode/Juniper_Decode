package org.firstinspires.ftc.teamcode.Spindex;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

public class IntakeMotorFSM {
    public enum states {
        INTAKING,
        EJECTING,
        OFF
    }
//..
    public MotorWrapper intakeMotor;
    public states state;
    private Telemetry telemetry;

    public IntakeMotorFSM(HWMapSpindex hwMap, Telemetry telemetry) {
        this.intakeMotor = new MotorWrapper(hwMap.getIntakeMotor(), false, 1, 537.7);
        this.telemetry = telemetry;
        this.state = states.INTAKING; //default for now
    }

    public void updateState() {
        switch (state) {
            case INTAKING:
                intakeMotor.set(0.5);
                break;

            case EJECTING:
                intakeMotor.set(-0.3);
                intakeMotor.brakeModeMotor();
                break;

            case OFF:
                intakeMotor.set(0);
                intakeMotor.brakeModeMotor();
                break;
        }

        telemetry.addData("Intake Motor FSM State", state);
    }
}