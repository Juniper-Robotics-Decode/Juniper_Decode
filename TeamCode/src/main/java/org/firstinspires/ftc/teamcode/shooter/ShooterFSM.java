package org.firstinspires.ftc.teamcode.shooter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Pinpoint;

public class ShooterFSM {

    public enum States{
        PREPARING_TO_SHOOT,
        PREPARED_TO_SHOOT,
        TOGGLING_FLYWHEEL,
        TOGGLED_FLYWHEEL
    }

    private FlywheelFSM flywheelFSM;
    private TurretFSM turretFSM;
    private PitchFSM pitchFSM;
    private PositionFSM positionFSM;
    private States state;
    private boolean flywheelStopping = false;

    private Telemetry telemetry;
    public ShooterFSM (HWMap hardwareMap, Telemetry telemetry, Pinpoint pinpoint) {
        flywheelFSM = new FlywheelFSM(hardwareMap,telemetry);
        turretFSM = new TurretFSM(hardwareMap,telemetry);
        pitchFSM = new PitchFSM(hardwareMap,telemetry);
        positionFSM = new PositionFSM(hardwareMap,telemetry, pinpoint);
        this.telemetry = telemetry;
        state = States.PREPARING_TO_SHOOT;
    }

    public void updateState(boolean bPress) {
        flywheelFSM.updateState();
        turretFSM.updateState();
        pitchFSM.updateState();
        positionFSM.updateState();
        findTargetState(bPress);

        switch (state) {
            case PREPARING_TO_SHOOT:
                if(!flywheelStopping) {
                    flywheelFSM.setTargetVelocityRPM(positionFSM.getFlywheelTargetVelocityRPM());
                }
                turretFSM.setTargetAngle(positionFSM.getTurretError());
                pitchFSM.setTargetAngle(positionFSM.getPitchTargetAngle());
                if(flywheelFSM.AT_TARGET_VELOCITY() && turretFSM.ALIGNED() && pitchFSM.ALIGNED()) {
                    state = States.PREPARED_TO_SHOOT;
                }
                break;
            case TOGGLING_FLYWHEEL:
                if(flywheelFSM.STOPPED()) {
                    flywheelFSM.setTargetVelocityRPM(positionFSM.getFlywheelTargetVelocityRPM());
                    flywheelStopping = false;
                }
                else {
                    flywheelFSM.setTargetVelocityRPM(0);
                    flywheelStopping = true;
                }

                if(flywheelFSM.AT_TARGET_VELOCITY() || flywheelFSM.STOPPED()) {
                    state = States.TOGGLED_FLYWHEEL;
                }
                break;
        }
    }

    public void findTargetState(boolean bPress) {
        if(bPress) {
            state = States.TOGGLING_FLYWHEEL;
        }
        else if(!(state == States.TOGGLING_FLYWHEEL)) {
            state = States.PREPARING_TO_SHOOT;
        }
    }

    public void log() {
        telemetry.addData("shooter state", state);
        flywheelFSM.log();
        turretFSM.log();
        pitchFSM.log();
        positionFSM.log();
    }
}
