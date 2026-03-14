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

    public RollerFSM Roller;
    public IntakeServoFSM Servo;

    private State currentState = State.RAMPING_UP_TO_INTAKE;
    private Telemetry telemetry;
    Timing.Timer autoReverseTimer;
    boolean Last_D_Pad_Up_Press, Last_D_Pad_Left_Press, Last_D_Pad_Down_Press = false;

    TransferFSM transferFSM;

    Logger logger;

    public IntakeFSM(HWMap hardwareMap, Telemetry telemetry, Logger logger) {
        autoReverseTimer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
        Roller = new RollerFSM(hardwareMap, telemetry,logger);
        Servo = new IntakeServoFSM(hardwareMap, telemetry);
        this.telemetry = telemetry;
        this.logger = logger;
        telemetry.addData("Current Intake state", currentState);
    }

    public void updateState(boolean D_Pad_Up_Press, boolean D_Pad_Left_Press, boolean D_Pad_Down_Press) {
        Roller.updateState();
        Servo.updateState((D_Pad_Down_Press && !Last_D_Pad_Down_Press));

        findTargetState((D_Pad_Up_Press && !Last_D_Pad_Up_Press), (D_Pad_Left_Press && !Last_D_Pad_Left_Press));
        switch (currentState) {

            case RAMPING_UP_TO_INTAKE:
                Roller.intake();
                if (Roller.INTAKING()) {
                    currentState = State.READY_TO_INTAKE;
                    Servo.MoveDown();
                }
                break;

            case STOPPING:
                Roller.stop();
                if (Roller.STOPPED()) {
                    currentState = State.STOPPED;
                    Servo.MoveDown();
                }

                break;

            case RAMPING_UP_TO_EJECT:
                Roller.eject();
                if (Roller.EJECTING()) {
                    currentState = State.EJECTING;
                    Servo.MoveUp();
                }
                break;


        }

        Last_D_Pad_Left_Press = D_Pad_Left_Press;
        Last_D_Pad_Down_Press = D_Pad_Down_Press;
        Last_D_Pad_Up_Press = D_Pad_Up_Press;

        telemetry.addData("Intake Current State", currentState);
    }


    public void findTargetState(boolean D_Pad_Up_Press, boolean D_Pad_Left_Press) {

        if (D_Pad_Up_Press && (currentState == State.READY_TO_INTAKE || currentState == State.STOPPED || currentState == State.RAMPING_UP_TO_INTAKE)) {
            currentState = State.RAMPING_UP_TO_EJECT;

        } else if (D_Pad_Up_Press && (currentState == State.RAMPING_UP_TO_EJECT || currentState == State.EJECTING)) {
            currentState = State.RAMPING_UP_TO_INTAKE;

        }

        if (D_Pad_Left_Press && (currentState == State.READY_TO_INTAKE || currentState == State.EJECTING)) {
            currentState = State.STOPPING;
        } else if (D_Pad_Left_Press && (currentState == State.STOPPED || currentState == State.STOPPING)) {
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