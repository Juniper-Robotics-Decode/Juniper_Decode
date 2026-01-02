package org.firstinspires.ftc.teamcode.shooter;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;
import org.firstinspires.ftc.teamcode.core.RobotSettings;


@Config
public class TurretFSM {
    public enum States{
        ALIGNING,
        ALIGNED
    }

    private MotorWrapper turretMotor;
    private States state;
    private double targetAngle;

    private PIDController pidController;
    public static double TOLERANCE = 3;
    public static double P=-0.015, I=0.0, D=0, F=0.2;
    public static double gearRatio = 16.0/109.0;

    public static double UPPER_HARD_STOP = 0;
    public static double LOWER_HARD_STOP = -90;

    public static double POWER_CAP = 1;

    public static double TURRET_OFFSET = 3;


    int i = 0;
    
    Telemetry telemetry;

    public TurretFSM(HWMap hwMap, Telemetry telemetry) {
        turretMotor = new MotorWrapper(hwMap.getTurretMotor(),false,gearRatio, false);
        turretMotor.resetEncoder();
        state = States.ALIGNING;
        pidController = new PIDController(P,I,D);
        pidController.setTolerance(TOLERANCE);
        this.telemetry = telemetry;
    }

    public void updateState(){
        updatePID();
        if(pidController.atSetPoint()) {
            state = States.ALIGNED;
        }
        else {
            state = States.ALIGNING;
        }
    }


    public void updatePID() {
        pidController.setPID(P,I,D);
        pidController.setTolerance(TOLERANCE);
        turretMotor.readPosition();
        if(targetAngle > UPPER_HARD_STOP) {
            targetAngle = UPPER_HARD_STOP;
        }
        else if (targetAngle < LOWER_HARD_STOP) {
            targetAngle = LOWER_HARD_STOP;
        }

/*
        double delta = angleDelta(turretMotor.getScaledPos(), targetAngle);
        double sign = angleDeltaSign(turretMotor.getScaledPos(), targetAngle);*/
        double currentPos = turretMotor.getScaledPos();
        double error = targetAngle - currentPos;
        telemetry.addData("Error", error);

        double power = pidController.calculate(currentPos,targetAngle);
        if(Math.abs(error) >= TOLERANCE) {
            power = power + (F * Math.signum(error));
        }
        if(Math.abs(power) > POWER_CAP) {
            double signPower = Math.signum(power);
            power = signPower*POWER_CAP;
        }
        turretMotor.set(power);
    }

    private double angleDelta(double measuredAngle, double targetAngle) {
        return Math.min(normalizeDegrees(measuredAngle - targetAngle), 360 - normalizeDegrees(measuredAngle - targetAngle));
    }

    private double angleDeltaSign(double measuredAngle, double targetAngle) {
        return -(Math.signum(normalizeDegrees(targetAngle - measuredAngle) - (360 - normalizeDegrees(targetAngle - measuredAngle))));
    }

    public void setTargetAngle(double turretError) {
        if(i >= 0) {
            if(PositionFSM.sensor == PositionFSM.Sensor.PINPOINT) {
                targetAngle = -turretError;
            }
            else {
                targetAngle = turretMotor.getScaledPos() + turretError + TURRET_OFFSET;
            }
            i = 0;
        }
        else {
            targetAngle = targetAngle;
            i++;
        }

        telemetry.addData("Turret target angle counter", i);
    }
    
    public boolean ALIGNED() {
        return state == States.ALIGNED;
    }

    public void log() {
        telemetry.addData("turret state", state);
        telemetry.addData("turret target angle", targetAngle);
        telemetry.addData("turret current angle", turretMotor.getScaledPos());
    }

    public double getCurrentAngle() {
        return turretMotor.getScaledPos();
    }


}
