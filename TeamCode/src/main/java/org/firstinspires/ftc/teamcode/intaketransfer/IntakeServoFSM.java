package org.firstinspires.ftc.teamcode.intaketransfer;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;

import java.util.concurrent.TimeUnit;

@Config
public class IntakeServoFSM {

    public enum State {
        MOVING_TO_POSITION,
        AT_UP,
        AT_DOWN,
    }

    private Telemetry telemetry;
    private ServoWrapper intakeServo;
    public State currentState;
    public static double targetPosition = .6;
    public static double positionUp = 1;
    public static double positionDown = .6;
    static Timing.Timer transferPostitionTimer;

    public IntakeServoFSM(HWMap intaketransferhwmap, Telemetry telemetry) {
        intakeServo = new ServoWrapper(intaketransferhwmap.getTransferServo());
        transferPostitionTimer = new Timing.Timer(1, TimeUnit.SECONDS); // Original length 1000
        this.telemetry = telemetry;
        currentState = State.AT_DOWN;
    }

    public void updateState() {

        telemetry.addData("Current Position ", intakeServo.getPosition());
        telemetry.addData("Elapsed Time ", transferPostitionTimer.elapsedTime());
        telemetry.addData("Target Position ", targetPosition);
        telemetry.addData("ServoFSM State ", currentState);


        double percentError = Math.abs((intakeServo.getPosition() - targetPosition) / targetPosition);
        telemetry.addData("Percent Error", percentError);
        if (!transferPostitionTimer.isTimerOn() && percentError >= 0.001) {
            currentState = State.MOVING_TO_POSITION;
            intakeServo.setPosition(targetPosition);
            transferPostitionTimer.start();
        }

        if (targetPosition == positionUp) {
            currentState = State.AT_UP;
        }

        if (targetPosition == positionDown) {
            currentState = State.AT_DOWN;
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
}