package org.firstinspires.ftc.teamcode.intake;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;

import java.util.concurrent.TimeUnit;

public class IntakeFSM {

    enum State {
        RAMPING_UP_TO_INTAKE,
        RAMPING_UP_TO_EJECT,
        STOPPING,
        READY_TO_INTAKE,
        EJECTING,
        STOPPED,
        REMOVING_JAM
    }

    private RollerFSM Roller;
    private State currentState = State.RAMPING_UP_TO_INTAKE;
    private Telemetry telemetry;
    Timing.Timer autoReverseTimer;

    public IntakeFSM(HWMap hardwareMap, Telemetry telemetry) {
        autoReverseTimer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
        Roller = new RollerFSM(hardwareMap, telemetry);
        this.telemetry = telemetry;
    }

    public void updateState(boolean YPress, boolean D_Pad_Left_Press) {
        Roller.updateState();
        findTargetState(YPress, D_Pad_Left_Press);
        switch (currentState) {

            case RAMPING_UP_TO_INTAKE:
                Roller.intake();
                if (Roller.INTAKING()) {
                    currentState = State.READY_TO_INTAKE;
                }
                break;

            case STOPPING:
                Roller.stop();
                if (Roller.STOPPED()) {
                    currentState = State.STOPPED;
                }
                break;

            case RAMPING_UP_TO_EJECT:
                Roller.eject();
                if (Roller.EJECTING()) {
                    currentState = State.EJECTING;
                }
                break;

            case REMOVING_JAM:
                Roller.eject();
                if (Roller.EJECTING()) {
                    currentState = State.RAMPING_UP_TO_INTAKE;
                }
        }
        telemetry.addData("Intake Current State ", currentState);
    }

    public void findTargetState(boolean YPress, boolean D_Pad_Left_Press) {

        if (YPress && (currentState == State.READY_TO_INTAKE || currentState == State.STOPPED)) {
            currentState = State.RAMPING_UP_TO_EJECT;
        } else if (YPress && currentState == State.EJECTING) {
            currentState = State.RAMPING_UP_TO_INTAKE;
        }

        if (Roller.JAMMED()) {
            currentState = State.REMOVING_JAM;
        }

        if (Roller.STOPPED() && currentState == State.READY_TO_INTAKE){
            currentState = State.RAMPING_UP_TO_EJECT;
        }

        if (D_Pad_Left_Press && (currentState == State.READY_TO_INTAKE || currentState == State.EJECTING)) {
            currentState = State.STOPPING;
        } else if (D_Pad_Left_Press && currentState == State.STOPPED) {
            currentState = State.RAMPING_UP_TO_INTAKE;

        }
    }
}