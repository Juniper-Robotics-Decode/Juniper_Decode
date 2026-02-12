package org.firstinspires.ftc.teamcode.shooter;

import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.shooter.wrappers.LimelightCamera;
import org.firstinspires.ftc.teamcode.core.Pinpoint;

import java.util.function.DoubleSupplier;

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

    public enum Sensor {
        LIMELIGHT,
        PINPOINT
    }

    private States state;
    public static Sensor sensor;
    private LimelightCamera limelightCamera;
    private Pinpoint pinpoint;
    private InterpLUT velocityMap;

    private double defaultFlywheelVelocity = 2500;
    private double flywheelTargetVelocityRPM;
    private double pitchTargetAngle;
    private double turretError;

    private double LIMELIGHT_FORWARD_OFFSET = 0; // TODO: add offset for pinpoint instead
    private double PINPOINT_OFFSET = 0;
    private double threshold1 = 2.5, threshold2 = 3;

    private double SENSOR_CHOICE_THRESHOLD = 2;
    private double RELOCALIZATION_TRHESHOLD = 0.1;
    private boolean rumbleNotification = false;
    public static double CAMERA_DISTANCE_FROM_CENTER = 0.1;

    private DoubleSupplier turretAngleProvider;

    private Logger logger;

    public PositionFSM(HWMap hwMap, Logger logger, Pinpoint pinpoint, DoubleSupplier turretAngleProvider) {
        limelightCamera = new LimelightCamera(hwMap.getLimelight(), logger);
        this.pinpoint = pinpoint;
        state = States.NO_VALID_TARGET;
        sensor = Sensor.PINPOINT;
        this.turretAngleProvider = turretAngleProvider;
        createVelocityMap();
        this.logger = logger;
    }

    public void updateState() {
        limelightCamera.update();
        chooseSensor();

        if (limelightCamera.hasTarget() || pinpoint.pinpointReady()) {
            if (sensor == Sensor.LIMELIGHT) {
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
            } else if (sensor == Sensor.PINPOINT) {
                if (pinpoint.pinpointReady()) {
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
                } else {
                    state = States.NO_VALID_TARGET;
                }
            }
        } else {
            state = States.NO_VALID_TARGET;
        }

        //TODO: add if turret velocity under threshold and drive velocity under threshold then relocalize at all times
/*
        if(RobotSettings.distanceMethod.equals(RobotSettings.DistanceMethod.LIMELIGHT_AND_PINPOINT)) {
            if(limelightCamera.hasTarget() && pinpoint.pinpointReady()) {
                relocalize();
            }
        }*/
    }

    private void createVelocityMap() {
        velocityMap = new InterpLUT();

        // distance (m) , velocity (rpm)

        velocityMap.add(1.24, 3100);
        velocityMap.add(1.6, 3100);
        velocityMap.add(2.11, 3500);
        velocityMap.add(2.8, 3900);

        velocityMap.createLUT();

    }

    public void findFlywheelTargetVelocity(double distance_m) {
        if (distance_m < 1.24 || distance_m > 2.8 || Double.isNaN(distance_m)) {
            flywheelTargetVelocityRPM = defaultFlywheelVelocity;
        } else {
            if (sensor == Sensor.LIMELIGHT) {
                flywheelTargetVelocityRPM = velocityMap.get(distance_m + LIMELIGHT_FORWARD_OFFSET);
            } else {
                flywheelTargetVelocityRPM = velocityMap.get(distance_m + PINPOINT_OFFSET);
            }
        }

    }

    public void findPitchTargetAngle() {
        if (state == States.NO_VALID_TARGET) {
            return;
        }
        pitchTargetAngle = state.getTargetAngle();
    }

    public void findTurretError(double error) {
        if (state == States.NO_VALID_TARGET || Double.isNaN(error)) {
            turretError = 0;
        } else {
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

    private void chooseSensor() {
        if (RobotSettings.distanceMethod.equals(RobotSettings.DistanceMethod.LIMELIGHT_ONLY)) {
            sensor = Sensor.LIMELIGHT;
        } else if (RobotSettings.distanceMethod.equals(RobotSettings.DistanceMethod.PINPOINT_ONLY)) {
            sensor = Sensor.PINPOINT;
        } else if (RobotSettings.distanceMethod.equals(RobotSettings.DistanceMethod.LIMELIGHT_AND_PINPOINT)) {
            if (!pinpoint.pinpointReady()) {
                sensor = Sensor.LIMELIGHT;
            } else {
                sensor = Sensor.PINPOINT;
            }
            if (Math.abs(pinpoint.getGoalDistance() - limelightCamera.getFlatDistance()) > 0.1 && pinpoint.pinpointReady() && limelightCamera.hasTarget()) {
                rumbleNotification = true;
            } else {
                rumbleNotification = false;
            }
        }

    }

    public void relocalize() {
        //TODO: add 20 degree pitch and add 90 degree roll
        double cameraAbsoluteHeading = pinpoint.getHeading() - turretAngleProvider.getAsDouble();
        double vectorAbsoluteHeading = cameraAbsoluteHeading - limelightCamera.getTy();
        double vectorMagnitude = limelightCamera.getFlatDistance();

        double xCam = RobotSettings.alliance.getGoalPos().getX(DistanceUnit.METER) - (vectorMagnitude * (Math.cos(Math.toRadians(vectorAbsoluteHeading))));
        double yCam = RobotSettings.alliance.getGoalPos().getY(DistanceUnit.METER) - (vectorMagnitude * (Math.sin(Math.toRadians(vectorAbsoluteHeading))));

        double xRobot = xCam + (CAMERA_DISTANCE_FROM_CENTER * (Math.cos(Math.toRadians(cameraAbsoluteHeading))));
        double yRobot = yCam + (CAMERA_DISTANCE_FROM_CENTER * (Math.sin(Math.toRadians(cameraAbsoluteHeading))));

        Pose2D newPos = new Pose2D(DistanceUnit.METER, xRobot, yRobot, AngleUnit.DEGREES, pinpoint.getHeading());
        pinpoint.setPosition(newPos);

        logger.log("RELOCALIZATION", "", Logger.LogLevels.DEBUG);
        logger.log("Cam absolute Heading", cameraAbsoluteHeading, Logger.LogLevels.DEBUG);
        logger.log("Vector Heading", vectorAbsoluteHeading, Logger.LogLevels.DEBUG);
        logger.log("Vector Dist", vectorMagnitude, Logger.LogLevels.DEBUG);
        logger.log("Cam X", xCam, Logger.LogLevels.DEBUG);
        logger.log("Cam Y", yCam, Logger.LogLevels.DEBUG);
        logger.log("Robot X", xRobot, Logger.LogLevels.DEBUG);
        logger.log("Robot Y", yRobot, Logger.LogLevels.DEBUG);
    }


    public void log() {
        logger.log("----------POSITION FSM LOG----------", "", Logger.LogLevels.DEBUG);
        logger.log("position FSM state", state, Logger.LogLevels.PRODUCTION);
        logger.log("Current shooting Sensor", sensor, Logger.LogLevels.PRODUCTION);
        logger.log("Flywheel Target", flywheelTargetVelocityRPM, Logger.LogLevels.DEBUG);
        logger.log("Pitch Target", pitchTargetAngle, Logger.LogLevels.DEBUG);
        logger.log("Turret Error", turretError, Logger.LogLevels.DEBUG);


        logger.log("----------LIMELIGHT LOG----------", "", Logger.LogLevels.DEBUG);
        logger.log("X", limelightCamera.getX(), Logger.LogLevels.DEBUG);
        logger.log("Y", limelightCamera.getY(), Logger.LogLevels.DEBUG);
        logger.log("Z", limelightCamera.getZ(), Logger.LogLevels.DEBUG);
        logger.log("Flat Distance", limelightCamera.getFlatDistance(), Logger.LogLevels.DEBUG);
        logger.log("tx", limelightCamera.getTx(), Logger.LogLevels.DEBUG);
        logger.log("ty", limelightCamera.getTy(), Logger.LogLevels.DEBUG);
        logger.log("Has target", limelightCamera.hasTarget(), Logger.LogLevels.DEBUG);
        logger.log("----------LIMELIGHT LOG----------", "", Logger.LogLevels.DEBUG);

        logger.log("----------PINPOINT LOG----------", "", Logger.LogLevels.DEBUG);
        logger.log("Goal Distance", pinpoint.getGoalDistance(), Logger.LogLevels.DEBUG);
        logger.log("pinpoint heading error", pinpoint.getHeadingErrorTrig(), Logger.LogLevels.DEBUG);
        logger.log("pinpoint ready", pinpoint.pinpointReady(), Logger.LogLevels.DEBUG);
        logger.log("----------PINPOINT LOG----------", "", Logger.LogLevels.DEBUG);

        logger.log("----------POSITION FSM LOG----------", "", Logger.LogLevels.DEBUG);
    }
}
