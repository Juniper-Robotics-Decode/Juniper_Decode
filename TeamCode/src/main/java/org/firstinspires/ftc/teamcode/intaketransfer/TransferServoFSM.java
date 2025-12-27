package org.firstinspires.ftc.teamcode.intaketransfer;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;

import java.util.concurrent.TimeUnit;

@Config
public class TransferServoFSM {

    public enum State {
        MOVING_TO_POSITION,
        AT_UP,
        AT_DOWN,
    }

    private Telemetry telemetry;
    private ServoWrapper transferServo;
    public State currentState;
    public static double targetPosition = .29;
    public static double positionUp = .51; // Old transfer .85
    public static double positionDown = .29; // Old transfer .55
    static Timing.Timer postitionTimer;

    public TransferServoFSM(HWMap intaketransferhwmap, Telemetry telemetry) {
        transferServo = new ServoWrapper(intaketransferhwmap.getTransferServo());
        postitionTimer = new Timing.Timer(1, TimeUnit.SECONDS); // Original length 1000
        this.telemetry = telemetry;
        currentState = State.AT_DOWN;
    }

    public void updateState() {

        telemetry.addData("Current Position ", transferServo.getPosition());
        telemetry.addData("Elapsed Time ", postitionTimer.elapsedTime());
        telemetry.addData("Target Position ", targetPosition);
        telemetry.addData("ServoFSM State ", currentState);


        double percentError = Math.abs((transferServo.getPosition() - targetPosition) / targetPosition);
        telemetry.addData("Percent Error", percentError);
        if (!postitionTimer.isTimerOn() && percentError >= 0.001) {
            currentState = State.MOVING_TO_POSITION;
            transferServo.setPosition(targetPosition);
            postitionTimer.start();
        }

        if (postitionTimer.done()) {
            postitionTimer.pause();
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