package org.firstinspires.ftc.teamcode.intake;

import com.arcrobotics.ftclib.util.Timing;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.intaketransfer.IntakeServoFSM;
import org.firstinspires.ftc.teamcode.intaketransfer.TransferFSM;

import java.util.concurrent.TimeUnit;

public class IntakeFSM {

    enum State {
        RAMPING_UP_TO_INTAKE,
        RAMPING_UP_TO_EJECT,
        STOPPING,
        READY_TO_INTAKE,
        EJECTING,
        STOPPED,
    }

    private RollerFSM Roller;
    private IntakeServoFSM Servo;

    private State currentState = State.RAMPING_UP_TO_INTAKE;
    private Telemetry telemetry;
    Timing.Timer autoReverseTimer;

    TransferFSM transferFSM;

    Logger logger;

    public IntakeFSM(HWMap hardwareMap, Telemetry telemetry, TransferFSM transferFSM, Logger logger) {
        this.transferFSM = transferFSM;
        autoReverseTimer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
        Roller = new RollerFSM(hardwareMap, telemetry, logger);
        Servo = new IntakeServoFSM(hardwareMap, telemetry);
        this.telemetry = telemetry;
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
                    if (Servo.AT_UP()) {
                        Servo.MoveDown();
                    }
                }
                break;

            case STOPPING:
                Roller.stop();
                if (Roller.STOPPED()) {
                    currentState = State.STOPPED;
                    if (Servo.AT_UP()) {
                        Servo.MoveDown();
                    }
                }

                break;

            case RAMPING_UP_TO_EJECT:
                Roller.eject();
                if (Roller.EJECTING()) {
                    currentState = State.EJECTING;
                    Servo.MoveUp();
                    if (Servo.AT_UP()) {
                        Servo.MoveDown();
                    }
                }
                break;


        }
    }

    public void findTargetState(boolean YPress, boolean D_Pad_Left_Press) {

        if (YPress && (currentState == State.READY_TO_INTAKE || currentState == State.STOPPED || currentState == State.RAMPING_UP_TO_INTAKE)) {
            currentState = State.RAMPING_UP_TO_EJECT;

        } else if (YPress && (Roller.EJECTING())) {
            currentState = State.RAMPING_UP_TO_INTAKE;

        }

        if (D_Pad_Left_Press && (currentState == State.READY_TO_INTAKE || currentState == State.EJECTING)) {
            currentState = State.STOPPING;
        } else if (D_Pad_Left_Press && currentState == State.STOPPED) {
            currentState = State.RAMPING_UP_TO_INTAKE;
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

    public void log(){
        logger.log("Intake Current State ", currentState, Logger.LogLevels.DEBUG);
        Roller.log();

    }
}