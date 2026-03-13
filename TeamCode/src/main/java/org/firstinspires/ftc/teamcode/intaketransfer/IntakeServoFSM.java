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
    public static double targetPosition = .7;
    public static double positionUp = 1;
    public static double positionDown = .7;
    static Timing.Timer transferPostitionTimer;

    public IntakeServoFSM(HWMap intaketransferhwmap, Telemetry telemetry) {
        intakeServo = new ServoWrapper(intaketransferhwmap.getIntakeServo());
        transferPostitionTimer = new Timing.Timer(200, TimeUnit.MILLISECONDS); // Original length 1000
        this.telemetry = telemetry;
        currentState = State.AT_DOWN;
        intakeServo.setPosition(positionDown);
    }

    public void updateState() {
        telemetry.addData("intake servo current Position ", intakeServo.getPosition());
        telemetry.addData("Elapsed Time ", transferPostitionTimer.elapsedTime());
        telemetry.addData("Intake servo target Position ", targetPosition);
        telemetry.addData("intake servo State ", currentState);


        double percentError = Math.abs((intakeServo.getPosition() - targetPosition) / targetPosition);
        telemetry.addData("Percent Error", percentError);
        if (!transferPostitionTimer.isTimerOn() && percentError >= 0.001) {
            currentState = State.MOVING_TO_POSITION;
            intakeServo.setPosition(targetPosition);
            transferPostitionTimer.start();
        }


        if (targetPosition == positionUp ) {
            intakeServo.setPosition(positionUp);
            currentState = State.AT_UP;
        }

        if (targetPosition == positionDown) {
            intakeServo.setPosition(positionDown);
            currentState = State.AT_DOWN;
        }
    }

    public void MoveUp() {
        targetPosition = positionUp;
    }

    public void MoveDown() {
        targetPosition = positionDown;
    }

    public boolean AT_UP(){
        return currentState == State.AT_UP;
    }

    public boolean AT_DOWN(){
        return currentState == State.AT_DOWN;
    }
}