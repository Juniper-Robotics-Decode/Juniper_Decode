package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;


@Config
public class FlywheelFSM {
    public enum States{
        AT_TARGET_VELOCITY,
        STOPPED
    }

    public static double vP=3, vI=0, vD=0, vF = 0; // 0.3 p for just the first half

    public static double ks=0, kv=1.7, ka=0;  // 1.37 for just the first half

    public static double TOLERANCE = 100; // ticks


    public static double targetVelocityRPM;

    public static double targetVelocityTicks;

    private MotorWrapper flywheelMotor;

    private Telemetry telemetry;

    private States state;

    private boolean stopping = false;

    public FlywheelFSM(HWMap hwMap, Telemetry telemetry) {
        flywheelMotor = new MotorWrapper(hwMap.getFlywheelMotor(),true,1, false);
        this.telemetry = telemetry;
        state = States.STOPPED;
    }

    public void updateState() {
        updatePID();
        if(flywheelMotor.getVelocity() == 0) {
            state = States.STOPPED;
        }
        else if(atSetPoint()) {
            state = States.AT_TARGET_VELOCITY;
        }
    }


    public void updatePID() { // This method is used to update position every loop.
        if(targetVelocityRPM == 0 && !stopping) {
            stopping = true;
            targetVelocityRPM = flywheelMotor.getVelocity();
        }
        if(stopping) {
            targetVelocityRPM = targetVelocityRPM - 50;
        }
        if(targetVelocityRPM <= 0){
            targetVelocityRPM = 0;
            stopping = false;
        }
        flywheelMotor.readVelocity();
        flywheelMotor.setVelocityConstants(vP,vI,vD,ks,kv,ka);
        targetVelocityTicks = convertRPMToTicks(targetVelocityRPM);
        targetVelocityTicks = targetVelocityTicks;
        flywheelMotor.setVelocity(targetVelocityTicks);
    }

    private boolean atSetPoint() {
        return (flywheelMotor.getVelocity() <= targetVelocityTicks + TOLERANCE) && (flywheelMotor.getVelocity() >= targetVelocityTicks - TOLERANCE);
    }

    public void setTargetVelocityRPM(double targetVelocityRPM) {
        this.targetVelocityRPM = targetVelocityRPM;
    }

    private static double convertRPMToTicks(double RPMVelocity) {
        return (RPMVelocity*28)/60;
    }

    public boolean AT_TARGET_VELOCITY() {
        return state == States.AT_TARGET_VELOCITY;
    }

    public boolean STOPPED() {
        return state == States.STOPPED;
    }


    public void log() {
        telemetry.addData("flywheel stopping varialbe", stopping);
        telemetry.addData("Flywheel FSM state", state);
        telemetry.addData("Target Velocity RPM", targetVelocityRPM);
        telemetry.addData("Target Velocity Ticks", targetVelocityTicks);
        telemetry.addData("Current Velocity Corrected", flywheelMotor.getVelocity());
    }


}
