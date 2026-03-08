package org.firstinspires.ftc.teamcode.intaketransfer;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;


import java.util.concurrent.TimeUnit;

public class TransferFSM {

    public enum State {
        OPENING,
        OPENED,
        CLOSING,
        CLOSED
    }

    private State currentState = State.OPENED;
    private Telemetry telemetry;
    private GateFSM transferServoFSM;
    public static Timing.Timer autoMoveTimer;
    public static Timing.Timer upTimer;
    private double counter = 0;
    boolean lastRightBumper;
    private boolean hasCountedCurrentCycle = false;
    private Logger logger;

    public static long DOWN_TIME = 1000;
    public static long UP_TIME = 500;


    public TransferFSM(HWMap hardwareMap, Telemetry telemetry, Logger logger) {
        this.logger = logger;
        this.telemetry = telemetry;
        transferServoFSM = new GateFSM(hardwareMap, telemetry, logger);
        autoMoveTimer = new Timing.Timer(DOWN_TIME, TimeUnit.MILLISECONDS);
        upTimer = new Timing.Timer(UP_TIME,TimeUnit.MILLISECONDS);
    }

    public void updateState(boolean Right_Bumper) {
        transferServoFSM.updateState();
        findTargetState(Right_Bumper);

        switch (currentState) {
            case CLOSING:
                transferServoFSM.MoveUp();
                if(transferServoFSM.AT_UP()) {
                    currentState = State.CLOSED;
                }
                /*if(transferServoFSM.AT_DOWN()) {
                    if(!upTimer.isTimerOn()) {
                        upTimer.start();
                    }
                    upTimer.start();
                    hasCountedCurrentCycle = false;
                    if(autoMoveTimer.done() || counter == 0) {
                        autoMoveTimer.pause();
                        transferServoFSM.MoveUp();
                    }
                    if(counter >= 0) {
                        counter = 0;
                        currentState = State.TRANSFERED;
                    } else {
                        if(autoMoveTimer.done() || counter == 0) {
                            autoMoveTimer.pause();
                            transferServoFSM.MoveUp();
                        }
                    }
                }
                else if(transferServoFSM.AT_UP() && upTimer.done()) {
                    upTimer.pause();
                    transferServoFSM.MoveDown();
                    if(!autoMoveTimer.isTimerOn()) {
                        autoMoveTimer.start();
                    }
                    if (!hasCountedCurrentCycle) {
                        counter++;
                        hasCountedCurrentCycle = true;
                    }
                }*/
                break;
            case OPENING:
                transferServoFSM.MoveDown();
                if(transferServoFSM.AT_DOWN()) {
                    currentState = State.OPENED;
                }
                break;
        }
    }

    public void findTargetState(boolean Right_Bumper) {
        if(Right_Bumper) {
            currentState = State.OPENING;
        }
        else {
            currentState = State.CLOSING;
        }
        lastRightBumper = Right_Bumper;

    }

    public boolean TRANSFERING() {
        return currentState == State.CLOSING;
    }


    public void log() {
        logger.log("Transfer Current State ", currentState, Logger.LogLevels.PRODUCTION);
        transferServoFSM.log();
        logger.log("Auto Transfer Move Timer", autoMoveTimer.elapsedTime(), Logger.LogLevels.PRODUCTION);
    }

    public boolean TRANSFERED() {
        return currentState == State.CLOSED;
    }
}