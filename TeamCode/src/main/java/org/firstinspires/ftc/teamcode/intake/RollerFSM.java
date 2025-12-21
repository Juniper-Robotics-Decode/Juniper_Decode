package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

@Config
public class RollerFSM {
    public static DcMotorEx Roller;
    private Telemetry telemetry;
    public static State State; // If transfer servo is moving, eject and if intake has significant velocity drop because of third ball eject
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
        JAMMED,
        EJECTING_AT_FULL_SPEED,
    }

    public RollerFSM(HWMap hwMap, Telemetry telemetry) {
        intakeMotor = new MotorWrapper(hwMap.getIntakeMotor(), true, 1);
        this.telemetry = telemetry;
        State = State.STOPPED;
    }

    public void updateState() {

        if (intakeMotor.getVelocity() == 0) {
            State = State.STOPPED;
        }

        if (intakeMotor.getVelocity() > 2300) {
            State = State.INTAKING;
        }

        if (intakeMotor.getVelocity() < 0 && State != State.EJECTING) {
            State = State.EJECTING;
        }

        if (intakeMotor.getVelocity() < jammingVelocityThreshold && State == State.INTAKING && jammingCurrentThreshold > intakeMotor.getCurrent() && State != State.EJECTING_AT_FULL_SPEED) {
            State = State.JAMMED;
        }

        if (State == State.JAMMED && intakeMotor.getVelocity() < -1200) {
            State = State.EJECTING_AT_FULL_SPEED;
        }

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

    public void stop() {
        targetVelocity = stoppingTargetVelocity;
    }

    public void intake() {
        targetVelocity = intakingTargetVelocity;
    }

    public void eject() {
        targetVelocity = ejectingTargetVelocity;
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

    public boolean JAMMED() {
        return State == State.JAMMED;
    }

    public boolean FULL_SPEED_EJECT() {
        return State == State.EJECTING_AT_FULL_SPEED;
    }
}