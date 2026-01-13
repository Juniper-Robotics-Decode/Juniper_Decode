package org.firstinspires.ftc.teamcode.intaketransfer;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;


import java.util.concurrent.TimeUnit;

public class TransferFSM {

    public enum State {
        RESTING,
        AT_REST,
        TRANSFERING,
        TRANSFERED
    }

    private State currentState = State.AT_REST;
    private Telemetry telemetry;
    private GateFSM transferServoFSM;
    public static Timing.Timer autoMoveTimer;
    private double counter = 0;
    boolean lastRightBumper;
    private boolean hasCountedCurrentCycle = false;


    public TransferFSM(HWMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        transferServoFSM = new GateFSM(hardwareMap, telemetry);
        autoMoveTimer = new Timing.Timer(3, TimeUnit.SECONDS);
    }

    public void updateState(boolean Right_Bumper) {
        transferServoFSM.updateState();
        findTargetState(Right_Bumper);

        switch (currentState) {
            case TRANSFERING:
                if(transferServoFSM.AT_DOWN()) {
                    hasCountedCurrentCycle = false;
                    transferServoFSM.MoveUp();
                    if(counter >= 3) {
                        counter = 0;
                        currentState = State.TRANSFERED;
                    } else {
                        transferServoFSM.MoveUp();
                    }
                }
                else if(transferServoFSM.AT_UP()) {
                    transferServoFSM.MoveDown();
                    if (!hasCountedCurrentCycle) {
                        counter++;
                        hasCountedCurrentCycle = true;
                    }
                }
                break;
            case RESTING:
                transferServoFSM.MoveDown();
                if(transferServoFSM.AT_DOWN()) {
                    counter = 0;
                    currentState = State.AT_REST;
                }
                break;
        }
        telemetry.addData("Transfer Current State ", currentState);
        telemetry.addData("Auto Move Timer ", autoMoveTimer.elapsedTime());
    }

    public void findTargetState(boolean Right_Bumper) {
        if(Right_Bumper && !lastRightBumper) {
            currentState = State.TRANSFERING;
        }
        else if (currentState != State.TRANSFERING) {
            currentState = State.RESTING;
        }
        lastRightBumper = Right_Bumper;
/*
        if (autoMoveTimer.done() && transferServoFSM.AT_UP()) {
            autoMoveTimer.pause();
            currentState = State.MOVING_DOWN;
        }*/
    }
}