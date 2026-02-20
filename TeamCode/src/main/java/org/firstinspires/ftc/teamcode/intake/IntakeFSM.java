package org.firstinspires.ftc.teamcode.intake;

import com.arcrobotics.ftclib.util.Timing;


import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.intaketransfer.IntakeServoFSM;

import java.util.concurrent.TimeUnit;


public class IntakeFSM {

    enum State {
        RAMPING_UP_TO_INTAKE,
        RAMPING_UP_TO_EJECT,
        STOPPING,
        READY_TO_INTAKE,
        EJECTING,
        STOPPED,
        MOVING_DOWN,
        MOVING_UP,
        AT_UP,
        AT_DOWN
    }

    private Logger logger;
    private RollerFSM Roller;
    private IntakeServoFSM Servo;

    private State currentState = State.RAMPING_UP_TO_INTAKE;
    Timing.Timer autoReverseTimer;

    public IntakeFSM(HWMap hardwareMap, Logger logger) {
        autoReverseTimer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
        Roller = new RollerFSM(hardwareMap, logger);
        Servo = new IntakeServoFSM(hardwareMap, logger);
        this.logger = logger;
    }

    public void updateState(boolean YPress, boolean D_Pad_Left_Press) {
        Roller.updateState();

        findTargetState(YPress, D_Pad_Left_Press);
        switch (currentState) {

            case RAMPING_UP_TO_INTAKE:
                Roller.intake();
                if (Roller.INTAKING()) {
                    currentState = State.READY_TO_INTAKE;
                }

                break;

            case STOPPING:
                Roller.stop();
                if (Roller.STOPPED()) {
                    currentState = State.STOPPED;
                }


                break;

            case RAMPING_UP_TO_EJECT:
                Roller.eject();
                if (Roller.EJECTING()) {
                    currentState = State.EJECTING;
                }


                break;
            case MOVING_DOWN:
                Servo.MoveDown();
                if (Servo.AT_DOWN()){
                currentState = State.AT_DOWN;
            }

            case MOVING_UP:
                Servo.MoveUp();
                if (Servo.AT_UP()){
                    currentState = State.AT_UP;
                }
        }
    }

    public void findTargetState(boolean YPress, boolean D_Pad_Left_Press) {

        if (YPress && (currentState == State.READY_TO_INTAKE || currentState == State.STOPPED)) {
            currentState = State.RAMPING_UP_TO_EJECT;

        } else if (YPress && (currentState == State.EJECTING)) {
            currentState = State.RAMPING_UP_TO_INTAKE;

        }

        if (D_Pad_Left_Press && (currentState == State.READY_TO_INTAKE || currentState == State.EJECTING)) {
            currentState = State.STOPPING;
        } else if (D_Pad_Left_Press && currentState == State.STOPPED) {
            currentState = State.RAMPING_UP_TO_INTAKE;
        }

        if (Servo.AT_DOWN() && Roller.INTAKING() || Roller.EJECTING()) {
            currentState = State.MOVING_UP;
        }

        if (Servo.AT_UP() && Roller.INTAKING()){
            currentState = State.MOVING_UP;
        }

        /*
                                        JAMMING

        if (Roller.RAMPING_UP_TO_INTAKE() && currentState == State.EJECTING){
            currentState = State.RAMPING_UP_TO_INTAKE;
        }

        if (Roller.JAMMED()) {
            currentState = State.RAMPING_UP_TO_EJECT;
        }
                                                                                                    */
    }

    public void log() {
        if (currentState == State.READY_TO_INTAKE) {
            logger.log("<b><u><font color='green'>Intake Current State</font></u></b>", currentState, Logger.LogLevels.PRODUCTION);
        } else if (currentState == State.EJECTING) {
            logger.log("<b><u><font color='red'>Intake Current State</font></u></b>", currentState, Logger.LogLevels.PRODUCTION);
        }
        Roller.log();
        Servo.log();

    }
}