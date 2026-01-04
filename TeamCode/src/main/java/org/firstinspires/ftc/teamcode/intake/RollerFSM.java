package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;


@Config
public class RollerFSM {
    public static MotorEx Roller;
    private Telemetry telemetry;
    public static State State; // If transfer servo is moving, eject and if intake has signiciant veolcity drop becuase of third ball eject
    private final MotorWrapper intakeMotor;
    public static double kS = 0, kV = 1.2, kA = 0;
    public static double p = 0.15, i = 0, d = 0;
    private double currentVelocity;
    public static double targetVelocity = 2790;
    public static double stoppingTargetVelocity = 0;
    public static double intakingTargetVelocity = 2790;
    public static double ejectingTargetVelocity = -1400;


    public enum State {
        STOPPED,
        INTAKING,
        EJECTING,
    }


    public RollerFSM(HWMap hwMap, Telemetry telemetry) {
        intakeMotor = new MotorWrapper(hwMap.getIntakeMotor(), true, 1, 537.7);
        this.telemetry = telemetry;
        State = State.STOPPED;
    }

    public void updateState() {

        if (intakeMotor.getVelocity() == 0) {
            State = State.STOPPED;
        }

        if (intakeMotor.getVelocity() > 0) {
            State = State.INTAKING;
        }

        if (intakeMotor.getVelocity() < 0) {
            State = State.EJECTING;

        }


        intakeMotor.readVelocity();
        intakeMotor.setVelocityConstants(p, i, d, kS, kV, kA);
        updatePID();
        telemetry.addData("Roller FSM State ", State);
        telemetry.addData("Current Veolcity ", currentVelocity);
        telemetry.addData("Target Velocity ", targetVelocity);
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

}
