package org.firstinspires.ftc.teamcode.shooter;

import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.core.Pinpoint;

public class LauncherFSM {

    public enum States{
        PREPARING_TO_SHOOT,
        PREPARED_TO_SHOOT,
        TOGGLING_FLYWHEEL,
        TOGGLED_FLYWHEEL
    }

    private Logger logger;
    private FlywheelFSM flywheelFSM;
    private TurretFSM turretFSM;
    private PitchFSM pitchFSM;
    private PositionFSM positionFSM;
    private States state;
    private boolean flywheelStopping = false;


    public LauncherFSM(HWMap hardwareMap,Logger logger, Pinpoint pinpoint) {
        flywheelFSM = new FlywheelFSM(hardwareMap,logger);
        turretFSM = new TurretFSM(hardwareMap,logger);
        pitchFSM = new PitchFSM(hardwareMap,logger);
        positionFSM = new PositionFSM(hardwareMap,logger, pinpoint, turretFSM::getCurrentAngle);
        this.logger = logger;
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
        logger.log("<b><u><font color='red'>shooter state</font></u></b>", state, Logger.LogLevels.PRODUCTION);
        turretFSM.log();
        flywheelFSM.log();
        pitchFSM.log();
        positionFSM.log();
    }
}
