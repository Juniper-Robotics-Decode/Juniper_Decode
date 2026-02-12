package org.firstinspires.ftc.teamcode.intake;

import static org.firstinspires.ftc.teamcode.core.Logger.LogLevels.DEBUG;
import static org.firstinspires.ftc.teamcode.core.Logger.LogLevels.PRODUCTION;

import com.acmerobotics.dashboard.config.Config;


import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;


@Config
public class RollerFSM {
    public static State State;
    private final MotorWrapper intakeMotor;
    public static double kS = 0, kV = 2, kA = 0;
    public static double p = 0.15, i = 0, d = 0;
    private final Logger Logger;
    private double currentVelocity;
    public static double targetVelocity = 2790;
    public static double stoppingTargetVelocity = 0;
    public static double intakingTargetVelocity = 2790;
    public static double ejectingTargetVelocity = -1400;
    public static double jammingCurrentThreshold = 5;
    public static double jammingVelocityThreshold = 1000;

    public void log() {
        Logger.log("Roller FSM State ", State, PRODUCTION);
        Logger.log("Current Velocity ", currentVelocity, DEBUG);
        Logger.log("Target Velocity ", targetVelocity, DEBUG);
        Logger.log("Current Amount ", intakeMotor.getCurrent(), DEBUG);
    }

    public enum State {
        STOPPED,
        INTAKING,
        EJECTING,
//        JAMMED,
//        RAMPING_UP_TO_INTAKE,
//        RAMPING_UP_TO_EJECT
    }

    public RollerFSM(HWMap hwMap, Logger logger) {
        intakeMotor = new MotorWrapper(hwMap.getIntakeMotor(), true, 1, false);
        this.Logger = logger;
        State = State.INTAKING;

    }

    public void updateState() {

        if (currentVelocity == 0) {
            State = State.STOPPED;
        }

        if (currentVelocity > 1500) {
            State = State.INTAKING;
        }

        if (currentVelocity < -300) {
            State = State.EJECTING;
        }

      /*  if (0 > targetVelocity && currentVelocity < jammingVelocityThreshold && jammingCurrentThreshold > intakeMotor.getCurrent() && State != State.RAMPING_UP_TO_INTAKE) {
            State = State.JAMMED;
        }*/

        intakeMotor.readVelocity();
        intakeMotor.setVelocityConstants(p, i, d, kS, kV, kA);
        updatePID();
    }

    public void updatePID() {
        currentVelocity = intakeMotor.getVelocity();
        intakeMotor.setVelocity(targetVelocity);
    }

    public boolean STOPPED() {
        return State == State.STOPPED;
    }

    public boolean INTAKING() {
        return State == State.INTAKING;
    }

    public boolean EJECTING() {
        return State == State.EJECTING;
    }

    public void stop() {
        targetVelocity = stoppingTargetVelocity;
    }

    public void intake() {
        targetVelocity = intakingTargetVelocity;
        /* if (State != State.INTAKING) {
            State = State.RAMPING_UP_TO_INTAKE;
        }*/
    }

    public void eject() {
        targetVelocity = ejectingTargetVelocity;
        /*if (State != State.EJECTING) {
            State = State.RAMPING_UP_TO_EJECT;
        }*/

         /*public boolean JAMMED() {
        return State == State.JAMMED;
    }
    public boolean RAMPING_UP_TO_INTAKE(){
        return State == State.RAMPING_UP_TO_INTAKE;
    }*/

    }
}