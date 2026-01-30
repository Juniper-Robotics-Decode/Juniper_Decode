package org.firstinspires.ftc.teamcode.Spindex;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;
import org.firstinspires.ftc.teamcode.Spindex.SpindexFSM.modes;

public class spindexMotorFSM {
    // Physical locations only
    public enum Positions {
        POSITION_1,
        POSITION_2,
        POSITION_3
    }

    public final MotorWrapper spindexMotor;
    public Positions currentPos;

    private Telemetry telemetry;
    public int currentIndex;
    public int offset = 0;

    public spindexMotorFSM(HWMapSpindex hwMap, Telemetry telemetry, MotorWrapper sharedMotor) {
        this.spindexMotor = sharedMotor;
        this.telemetry = telemetry;
        this.currentPos = Positions.POSITION_1;
    }

    public void spindexOffset(modes mode){
        if(mode == modes.SHOOTING){
            offset = 60;
        }
        else if(mode == modes.INTAKNG){
            offset = 0;
        }
    }

    public void updateState() {
        double a = Math.abs(spindexMotor.getAngle() + offset);
        double relativePositionInRotation = a % 360;

        if (relativePositionInRotation < 120) {
            currentIndex = 1;
            currentPos = Positions.POSITION_1;
        } else if (relativePositionInRotation < 240) {
            currentIndex = 2;
            currentPos = Positions.POSITION_2;
        } else {
            currentIndex = 3;
            currentPos = Positions.POSITION_3;
        }

        telemetry.addData("Spindex Position", currentPos);
        telemetry.addData("Spindex Index", currentIndex);
    }

    public boolean atPosition1() {
        return currentPos == Positions.POSITION_1;
    }

    public boolean atPosition2() {
        return currentPos == Positions.POSITION_2;
    }

    public boolean atPosition3() {
        return currentPos == Positions.POSITION_3;
    }

}