package org.firstinspires.ftc.teamcode.shooter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.core.HWMap;
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

    private FlywheelFSM flywheelFSM;
    private TurretFSM turretFSM;
    private PitchFSM pitchFSM;
    private PositionFSM positionFSM;
    private Pinpoint pinpoint;
    private States state;
    private boolean flywheelStopping = false;

    private Telemetry telemetry;
    public LauncherFSM(HWMap hardwareMap, Telemetry telemetry, Pinpoint pinpoint, RobotSettings robotSettings) {
        this.pinpoint = pinpoint;
        flywheelFSM = new FlywheelFSM(hardwareMap,telemetry);
        turretFSM = new TurretFSM(hardwareMap,telemetry);
        pitchFSM = new PitchFSM(hardwareMap,telemetry, flywheelFSM::getError);
        positionFSM = new PositionFSM(hardwareMap,telemetry, pinpoint, turretFSM::getCurrentAngle, robotSettings);
        this.telemetry = telemetry;
        state = States.PREPARING_TO_SHOOT;
    }

    public void updateState(boolean bPress, boolean dPadUp, boolean leftBumper) {
        flywheelFSM.updateState();
        turretFSM.updateState();
        pitchFSM.updateState();
        positionFSM.updateState();
        findTargetState(bPress, dPadUp);

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
            case RELOCALIZING:
                Pose2D newPos = positionFSM.getRobotPos();
                turretFSM.setTargetAngle(0);
                telemetry.addData("-----------------Good to Relocalize-----------", "");
                telemetry.addData("launcher new pos x", newPos.getX(DistanceUnit.METER));
                telemetry.addData("launcher new pos y", newPos.getY(DistanceUnit.METER));
                telemetry.addData("launcher new pos heading", newPos.getHeading(AngleUnit.DEGREES));
                if(turretFSM.ALIGNED() && leftBumper) {
                        if(newPos != null) {
                            pinpoint.setPosition(newPos);
                            state = States.RELOCALIZED;
                        }
                }
                break;
        }
    }

    public void findTargetState(boolean bPress, boolean dPadUp) {
        if(bPress) {
            state = States.TOGGLING_FLYWHEEL;
        }
        else if(dPadUp) {
            state = States.RELOCALIZING;
        }
        else if(!(state == States.TOGGLING_FLYWHEEL || state == States.RELOCALIZING)) {
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
