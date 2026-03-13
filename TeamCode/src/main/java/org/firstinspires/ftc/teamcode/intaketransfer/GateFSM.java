package org.firstinspires.ftc.teamcode.intaketransfer;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;


import java.util.concurrent.TimeUnit;

@Config
public class GateFSM {

    public enum State {
        MOVING_TO_POSITION,
        AT_UP,
        AT_DOWN
    }

    private Telemetry telemetry;
    private ServoWrapper transferServo;
    private Logger logger;
    public State currentState;
    public double currentPosition = 0;
    public static double targetPosition = 0.1;
    public static double positionUp = 0.1;
    public static double positionDown = 0;
    Timing.Timer timer;


    public GateFSM(HWMap intaketransferhwmap, Telemetry telemetry, Logger logger) {
        this.logger = logger;
        transferServo = new ServoWrapper(intaketransferhwmap.getTransferServo());
        timer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
        this.telemetry = telemetry;
        currentState = State.AT_DOWN;
    }

    public void updateState() {

        telemetry.addData("Current Position  ", transferServo.getPosition());
        telemetry.addData("Elapsed Time: ", timer.elapsedTime());
        telemetry.addData("Target Postion ", targetPosition);
        telemetry.addData("ServoFSM State ", currentState);


        double percentError = Math.abs((transferServo.getPosition() - targetPosition) / targetPosition);
        telemetry.addData("Percent Error", percentError);
        if (!timer.isTimerOn() && percentError >= 0.001) {
            currentState = State.MOVING_TO_POSITION;
            transferServo.setPosition(targetPosition);
            timer.start();
        }

        if (timer.done()) {
            timer.pause();
            if (targetPosition == positionUp) {
                currentState = State.AT_UP;
            }
            if (targetPosition == positionDown) {
                currentState = State.AT_DOWN;
            }
            if (targetPosition == positionDown) {
                currentState = State.AT_DOWN;
            }
            // else if(currentState = A)

        }
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
    public void log(){
        logger.log("gate state: ",currentState, Logger.LogLevels.PRODUCTION);
        logger.log("gate pos: ",currentPosition, Logger.LogLevels.PRODUCTION);
        logger.log("gate timer: ",timer.elapsedTime(), Logger.LogLevels.PRODUCTION);
    }
}