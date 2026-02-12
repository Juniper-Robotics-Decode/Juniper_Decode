package org.firstinspires.ftc.teamcode.intaketransfer;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;


import java.util.concurrent.TimeUnit;

@Config
public class GateFSM {

    public void log(){
        logger.log("Current Position ", transferServo.getPosition(), Logger.LogLevels.DEBUG);
        logger.log("Elapsed Time ", transferPostitionTimer.elapsedTime(), Logger.LogLevels.DEBUG);
        logger.log("Target Position ", targetPosition, Logger.LogLevels.PRODUCTION);
        logger.log("ServoFSM State ", currentState, Logger.LogLevels.PRODUCTION);
    }

    public enum State {
        MOVING_TO_POSITION,
        AT_UP,
        AT_DOWN,
    }

    private Logger logger;
    private ServoWrapper transferServo;
    public State currentState;
    public static double targetPosition = .7;
    public static double positionUp = 1;
    public static double positionDown = .7;
    static Timing.Timer transferPostitionTimer;

    public GateFSM(HWMap intaketransferhwmap, Logger logger) {
        transferServo = new ServoWrapper(intaketransferhwmap.getTransferServo());
        transferPostitionTimer = new Timing.Timer(1, TimeUnit.SECONDS); // Original length 1000
        this.logger = logger;
        currentState = State.AT_DOWN;
        transferServo.setPosition(positionDown);
    }

    public void updateState() {

        double percentError = Math.abs((transferServo.getPosition() - targetPosition) / targetPosition);
        if (!transferPostitionTimer.isTimerOn() && percentError >= 0.001) {
            currentState = State.MOVING_TO_POSITION;
            transferServo.setPosition(targetPosition);
            transferPostitionTimer.start();
        }




       if (transferPostitionTimer.done()) {
            transferPostitionTimer.pause();
            if (currentState == State.AT_UP){
                targetPosition = positionDown;
            }
        }

        if (targetPosition == positionUp) {
            currentState = State.AT_UP;
        }

        if (targetPosition == positionDown) {
            currentState = State.AT_DOWN;
        }
        // else if(currentState = A)
    }

    public boolean AT_DOWN() {
        return currentState == State.AT_DOWN;
    }

    public boolean AT_UP() {
        return currentState == State.AT_UP;
    }

    public void MoveUp() {
        targetPosition = positionUp;
    }

    public void MoveDown() {
        targetPosition = positionDown;
    }
}