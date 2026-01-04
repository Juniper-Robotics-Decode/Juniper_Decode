package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapTest;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

@Config
public class RollerFSM {
    private Telemetry telemetry;
    public static State State;
    private final MotorWrapper intakeMotor;
    public static double kS = 0, kV = 1.2, kA = 0;
    public static double p = 0.15, i = 0, d = 0;
    private double currentVelocity;
    public static double targetVelocity = 2790;
    public static double stoppingTargetVelocity = 0;
    public static double intakingTargetVelocity = 2790;
    public static double ejectingTargetVelocity = -1400;
    public static double jammingCurrentThreshold = 5;
    public static double jammingVelocityThreshold = 1000;

    public enum State {
        STOPPED,
        INTAKING,
        EJECTING,
//        JAMMED,
//        RAMPING_UP_TO_INTAKE,
//        RAMPING_UP_TO_EJECT
    }

    public RollerFSM(HWMapTest hwMap, Telemetry telemetry) {
        intakeMotor = new MotorWrapper(hwMap.getIntakeMotor(), true, 1, true);
        this.telemetry = telemetry;
        State = State.INTAKING;

    }

    public void updateState() {

        if (currentVelocity == 0) {
            State = State.STOPPED;
        }

        if (currentVelocity > 1500) {
            State = State.INTAKING;
        }

        if (currentVelocity < -500) {
            State = State.EJECTING;
        }

      /*  if (0 > targetVelocity && currentVelocity < jammingVelocityThreshold && jammingCurrentThreshold > intakeMotor.getCurrent() && State != State.RAMPING_UP_TO_INTAKE) {
            State = State.JAMMED;
        }*/

        intakeMotor.readVelocity();
        intakeMotor.setVelocityConstants(p, i, d, kS, kV, kA);
        updatePID();
        telemetry.addData("Roller FSM State ", State);
        telemetry.addData("Current Velocity ", currentVelocity);
        telemetry.addData("Target Velocity ", targetVelocity);
        telemetry.addData("Current Amount ", intakeMotor.getCurrent());
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