package org.firstinspires.ftc.teamcode.shooter;

import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.shooter.wrappers.LimelightCamera;
import org.firstinspires.ftc.teamcode.core.Pinpoint;

public class PositionFSM {

    public enum States {
        ZONE_1(13),   // TODO: check does pitch at 22.5 equal like almost 360?
        ZONE_2(22.5),
        ZONE_3(22.5),
        NO_VALID_TARGET;

        private double targetAngle;

        States(double angle) {
            this.targetAngle = angle;
        }

        States() {
            this.targetAngle = 0; // doesn't matter if 0 - never used
        }

        public double getTargetAngle() {
            return targetAngle;
        }
    }

    private States state;
    private LimelightCamera limelightCamera;
    private Pinpoint pinpoint;
    private InterpLUT velocityMap;

    private double defaultFlywheelVelocity;
    private double flywheelTargetVelocityRPM;
    private double pitchTargetAngle;
    private double turretError;

    private double LIMELIGHT_FORWARD_OFFSET = 0;
    private double threshold1 = 2.5, threshold2 = 3;

    private Telemetry telemetry;

    public PositionFSM(HWMap hwMap, Telemetry telemetry, Pinpoint pinpoint) {
        limelightCamera = new LimelightCamera(hwMap.getLimelight(), telemetry);
        this.pinpoint = pinpoint;
        state = States.NO_VALID_TARGET;
        createVelocityMap();
        this.telemetry = telemetry;
    }

    public void updateState() {
        limelightCamera.update();
        pinpoint.update();

        if(limelightCamera.hasTarget() && pinpoint.pinpointReady()) {
            if(RobotSettings.distanceMethod == RobotSettings.DistanceMethod.LIMELIGHT_ONLY && limelightCamera.hasTarget()) {
                if (limelightCamera.getFlatDistance() >= threshold2) {
                    state = States.ZONE_3;
                } else if (limelightCamera.getFlatDistance() >= threshold1) {
                    state = States.ZONE_2;
                } else {
                    state = States.ZONE_1;
                }
                findFlywheelTargetVelocity(limelightCamera.getFlatDistance());
                findPitchTargetAngle();
                findTurretError(limelightCamera.getTy());
            }
            else if(RobotSettings.distanceMethod == RobotSettings.DistanceMethod.PINPOINT_ONLY && pinpoint.pinpointReady()) {
                if (pinpoint.getGoalDistance() >= threshold2) {
                    state = States.ZONE_3;
                } else if (pinpoint.getGoalDistance() >= threshold1) {
                    state = States.ZONE_2;
                } else {
                    state = States.ZONE_1;
                }
                findFlywheelTargetVelocity(pinpoint.getGoalDistance());
                findPitchTargetAngle();
                findTurretError(pinpoint.getHeadingErrorTrig());
            }
        }
        else {
            state = States.NO_VALID_TARGET;
        }
    }

    private void createVelocityMap() {
        velocityMap = new InterpLUT();

        // distance (m) , velocity (rpm)

        velocityMap.add(1.24, 3500);
        velocityMap.add(1.6, 3500);
        velocityMap.add(2.11, 3600);
        velocityMap.add(2.8, 4000);

        velocityMap.createLUT();

    }

    public void findFlywheelTargetVelocity(double distance_m) {
        if(distance_m < 1.24 || distance_m > 2.8) {
            flywheelTargetVelocityRPM = defaultFlywheelVelocity;
        }
        else {
            flywheelTargetVelocityRPM = velocityMap.get(distance_m+LIMELIGHT_FORWARD_OFFSET);
        }

    }

    public void findPitchTargetAngle() {
        if(state == States.NO_VALID_TARGET) {
            return;
        }
        pitchTargetAngle = state.getTargetAngle();
    }

    public void findTurretError(double error) {
        if(state == States.NO_VALID_TARGET) {
            turretError = 0;
        }
        else {
            turretError = error;
        }
    }

    public double getFlywheelTargetVelocityRPM() {
        return flywheelTargetVelocityRPM;
    }

    public double getPitchTargetAngle() {
        return pitchTargetAngle;
    }

    public double getTurretError() {
        return turretError;
    }

    public void log() {
        telemetry.addLine("----------POSITION FSM LOG----------");
        telemetry.addData("position FSM state", state);
        telemetry.addData("Flywheel Target", flywheelTargetVelocityRPM);
        telemetry.addData("Pitch Target", pitchTargetAngle);
        telemetry.addData("Turret Error", turretError);


        telemetry.addLine("----------LIMELIGHT LOG----------");
        telemetry.addData("X", limelightCamera.getX());
        telemetry.addData("Y", limelightCamera.getY());
        telemetry.addData("Z", limelightCamera.getZ());
        telemetry.addData("Flat Distance", limelightCamera.getFlatDistance());
        telemetry.addData("tx",limelightCamera.getTx());
        telemetry.addData("ty", limelightCamera.getTy());
        telemetry.addData("Has target", limelightCamera.hasTarget());
        telemetry.addLine("----------LIMELIGHT LOG----------");

        telemetry.addLine("----------PINPOINT LOG----------");


        telemetry.addLine("----------PINPOINT LOG----------");
        telemetry.addData("Goal Distance", pinpoint.getGoalDistance());
        telemetry.addData("pinpoint heading error", pinpoint.getHeadingErrorTrig());
        telemetry.addData("pinpoint ready", pinpoint.pinpointReady());
        telemetry.addLine("----------POSITION FSM LOG----------");
    }

}
