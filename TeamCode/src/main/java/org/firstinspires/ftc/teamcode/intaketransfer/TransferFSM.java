package org.firstinspires.ftc.teamcode.intaketransfer;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;


import java.util.concurrent.TimeUnit;

public class TransferFSM {

    public enum State {
        MOVING_DOWN,
        MOVING_UP,
        START_TO_MOVE,
        STOPPING,
        AT_DOWN,
        AT_UP,
        MOVING_FORWARD,
        STOPPED,
        REVERSING,
    }

    private State currentState = State.START_TO_MOVE;
    private BeltFSM Belt;
    private Logger logger;
    private GateFSM gateFSM;
    public static Timing.Timer autoMoveTimer;


    public TransferFSM(HWMap hardwareMap, Logger logger) {
        Belt = new BeltFSM(hardwareMap, logger);
        this.logger = logger;
        gateFSM = new GateFSM(hardwareMap, logger);
        autoMoveTimer = new Timing.Timer(3, TimeUnit.SECONDS);
    }

    public void updateState(boolean D_Pad_Right_Press, boolean Right_Bumper) {
        gateFSM.updateState();
        findTargetState(D_Pad_Right_Press, Right_Bumper);

        switch (currentState) {

            case START_TO_MOVE:
                Belt.Move();
                if (Belt.MOVING()) {
                    currentState = State.MOVING_FORWARD;
                }
                break;

            case MOVING_DOWN:
                gateFSM.MoveDown();
                if (gateFSM.AT_DOWN()) {
                    currentState = State.AT_DOWN;
                }
                break;

            case MOVING_UP:
                gateFSM.MoveUp();
                if (gateFSM.AT_UP()) {
                    currentState = State.AT_UP;
                }
                break;

            case STOPPING:
                Belt.Stop();
                if (Belt.STOPPED()) {
                    currentState = State.STOPPED;
                }
                break;

            case REVERSING:
                Belt.Reverse();
                if (Belt.REVERSING()) {
                    currentState = State.REVERSING;
                }
        }
    }

    public void findTargetState(boolean D_Pad_Right_Press, boolean Right_Bumper) {
        if (D_Pad_Right_Press && Belt.MOVING()) {
            currentState = State.STOPPING;
        }

        if (D_Pad_Right_Press && Belt.STOPPED()) {
            currentState = State.START_TO_MOVE;
        }
        if (Right_Bumper && gateFSM.AT_UP()) {
            currentState = State.MOVING_DOWN;
        }

        if (currentState == State.AT_UP && gateFSM.AT_DOWN()) {
            currentState = State.AT_DOWN;
        }

        if (Right_Bumper && gateFSM.AT_DOWN()) {
            currentState = State.MOVING_UP;
        }

        /*
                    Transfer Automation

        if (autoMoveTimer.done() && gateFSM.AT_UP()) {
            autoMoveTimer.pause();
            currentState = State.MOVING_DOWN;
        }*/
    }

    public void log() {
        logger.log("Transfer Current State", currentState, Logger.LogLevels.PRODUCTION);
        gateFSM.log();
    }
}