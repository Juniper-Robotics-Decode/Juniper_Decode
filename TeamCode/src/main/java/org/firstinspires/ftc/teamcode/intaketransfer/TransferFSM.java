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

    // CHANGE 1: Initialize to true so the first 'false' command registers
    boolean lastRightBumper = true;

    private boolean hasCountedCurrentCycle = false;
    private Logger logger;

    public static long DOWN_TIME = 1000;
    public static long UP_TIME = 500;

    private boolean isAuto = false;

    public TransferFSM(HWMap hardwareMap, Telemetry telemetry, Logger logger, boolean isAuto) {
        this.logger = logger;
        this.telemetry = telemetry;
        transferServoFSM = new GateFSM(hardwareMap, telemetry, logger);
        autoMoveTimer = new Timing.Timer(DOWN_TIME, TimeUnit.MILLISECONDS);
        upTimer = new Timing.Timer(UP_TIME,TimeUnit.MILLISECONDS);
        this.isAuto = isAuto;
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
                break;
            case OPENING:
                transferServoFSM.MoveDown();
                if(transferServoFSM.AT_DOWN()) {
                    currentState = State.OPENED;
                }
                break;
        }
    }

    // CHANGE 2: Edge detection logic
    public void findTargetState(boolean Right_Bumper) {
        if (Right_Bumper != lastRightBumper) {
            if(Right_Bumper) {
                currentState = State.OPENING;
            }
            else {
                currentState = State.CLOSING;
            }
        }
        lastRightBumper = Right_Bumper;
    }

    public boolean CLOSED() {
        return currentState == State.CLOSED;
    }

    // CHANGE 3: Strictly wait for OPENED
    public boolean TRANSFERED() {
        return currentState == State.OPENED;
    }

    public void log() {
        logger.log("Transfer Current State ", currentState, Logger.LogLevels.PRODUCTION);
        transferServoFSM.log();
        logger.log("Auto Transfer Move Timer", autoMoveTimer.elapsedTime(), Logger.LogLevels.PRODUCTION);
    }

    public void setTransferTime(long t) {
        transferServoFSM.setTransfer_Time(t);
    }
}