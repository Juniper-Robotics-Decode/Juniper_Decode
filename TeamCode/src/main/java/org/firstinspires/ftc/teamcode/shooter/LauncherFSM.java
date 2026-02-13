package org.firstinspires.ftc.teamcode.shooter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.core.Pinpoint;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.shooter.wrappers.LimelightCamera;

public class LauncherFSM {

    public enum States{
        PREPARING_TO_SHOOT,
        PREPARED_TO_SHOOT,
        TOGGLING_FLYWHEEL,
        TOGGLED_FLYWHEEL,
        RELOCALIZING,
        RELOCALIZED
    }

    private static FlywheelFSM flywheelFSM;
    private static TurretFSM turretFSM;
    private static PitchFSM pitchFSM;
    private PositionFSM positionFSM;
    private Pinpoint pinpoint;
    private States state;
    private boolean flywheelStopping = false;

    private Logger logger;

    private Telemetry telemetry;

    boolean auto = false;

    public static double AUTO_RPM = 2700;

    boolean endOfAuto = false;

    public LauncherFSM(HWMap hardwareMap, Telemetry telemetry, Pinpoint pinpoint, RobotSettings robotSettings, Logger logger, boolean auto) {
        this.logger = logger;
        this.pinpoint = pinpoint;
        flywheelFSM = new FlywheelFSM(hardwareMap,telemetry, logger);
        turretFSM = new TurretFSM(hardwareMap,telemetry, logger);
        pitchFSM = new PitchFSM(hardwareMap,telemetry, flywheelFSM::getError, logger);
        positionFSM = new PositionFSM(hardwareMap,telemetry, pinpoint, turretFSM::getCurrentAngle, robotSettings, logger);
        this.telemetry = telemetry;
        state = States.PREPARING_TO_SHOOT;
        this.auto = auto;
    }

    public void updateState(boolean bPress, boolean yPress, boolean dPadUp2, boolean dPadDown2, boolean dPadLeft2, boolean dPadRight2, boolean yPress2, boolean aPress2, boolean bPress2, boolean xPress2, boolean leftBumper2, boolean rightBumper2) {
        flywheelFSM.updateState(bPress2,xPress2);
        turretFSM.updateState();
        pitchFSM.updateState(yPress2,aPress2);
        positionFSM.updateState(rightBumper2);
        findTargetState(bPress, yPress);

        switch (state) {
            case PREPARING_TO_SHOOT:
                if(!flywheelStopping && !auto) {
                    flywheelFSM.setTargetVelocityRPM(positionFSM.getFlywheelTargetVelocityRPM());
                }
                if(auto) {
                    flywheelFSM.setTargetVelocityRPM(AUTO_RPM);
                }


                if(endOfAuto) {
                   turretFSM.setTargetAngle(0, dPadUp2,dPadDown2,dPadLeft2,dPadRight2, leftBumper2);
                }
                else {
                    turretFSM.setTargetAngle(positionFSM.getTurretError(), dPadUp2,dPadDown2,dPadLeft2,dPadRight2, leftBumper2);
                }

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
            case RELOCALIZING:
                Pose2D newPos = positionFSM.relocalize();
                        if(newPos != null) {
                            pinpoint.setPosition(newPos);
                            state = States.RELOCALIZED;
                        }
                break;
        }
    }

    public void findTargetState(boolean bPress, boolean yPress) {
        if(bPress) {
            state = States.TOGGLING_FLYWHEEL;
        }
        else if(yPress) {
            state = States.RELOCALIZING;
        }
        else if(!(state == States.TOGGLING_FLYWHEEL || state == States.RELOCALIZING)) {
            state = States.PREPARING_TO_SHOOT;
        }

    }

    public void log() {
        logger.log("---------SHOOTER----------","", Logger.LogLevels.PRODUCTION);
        logger.log("shooter state", state, Logger.LogLevels.DEBUG);
        positionFSM.log();
        flywheelFSM.log();
        turretFSM.log();
        pitchFSM.log();
    }

    public static boolean launcherPrepared() {
        if(flywheelFSM.AT_TARGET_VELOCITY() && turretFSM.ALIGNED() && pitchFSM.ALIGNED()) {
            return true;
        }
        else {
            return false;
        }
    }

    public void setEndOfAuto(boolean endOfAuto) {
        this.endOfAuto = endOfAuto;
    }
}
