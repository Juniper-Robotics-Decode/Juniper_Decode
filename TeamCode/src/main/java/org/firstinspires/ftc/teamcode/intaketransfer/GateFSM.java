package org.firstinspires.ftc.teamcode.intaketransfer;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;

@Config
public class GateFSM {

    public enum State {
        AT_UP,
        AT_DOWN,
    }

    private Telemetry telemetry;
    private AxonServoWrapper transferServo;
    public State currentState;
    public static double positionUp = .38;
    public static double positionDown = 0.58;
    public static double targetPosition = positionDown;

    public static double UPPER_HARD_STOP = 1;
    public static double LOWER_HARD_STOP = 0;

    public static double TOLERANCE_DOWN = 0.07;

    public static double TOLERANCE_UP = 0.07;

    private Logger logger;

    public GateFSM(HWMap intaketransferhwmap, Telemetry telemetry, Logger logger) {
        this.logger = logger;
        transferServo = new AxonServoWrapper(intaketransferhwmap.getTransferServo(),intaketransferhwmap.getTransferEncoder(),false,false,0);
        this.telemetry = telemetry;
        currentState = State.AT_DOWN;
        transferServo.setPos(targetPosition);
    }

    public void updateState() {
        updatePID();
        if(transferServo.getLastReadPos() <= positionDown + TOLERANCE_DOWN && transferServo.getLastReadPos() >= positionDown - TOLERANCE_DOWN) {
            currentState = State.AT_DOWN;
        }
        else if (transferServo.getLastReadPos() >= positionUp) {
            currentState = State.AT_UP;
        }

        telemetry.addData("Transfer Current Position ", transferServo.getLastReadPos());
        telemetry.addData("Transfer Target Position ", targetPosition);
        telemetry.addData("ServoFSM State ", currentState);
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

    public void updatePID() {
        if(targetPosition > UPPER_HARD_STOP) {
            targetPosition = UPPER_HARD_STOP;
        }
        else if (targetPosition < LOWER_HARD_STOP) {
            targetPosition = LOWER_HARD_STOP;
        }
        transferServo.readPos();

        double error = targetPosition - transferServo.getLastReadPos();

        telemetry.addData("error", error);

        transferServo.setPos(targetPosition);
    }

    public void log(){
        logger.log("Current Position ", transferServo.getLastReadPos(), Logger.LogLevels.DEBUG);
        logger.log("Target Position ", targetPosition, Logger.LogLevels.DEBUG);
        logger.log("ServoFSM State ", currentState, Logger.LogLevels.DEBUG);
    }

}