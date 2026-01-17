package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.shooter.wrappers.LimelightCamera;
import org.firstinspires.ftc.teamcode.core.Pinpoint;

import java.util.function.DoubleSupplier;


@Config
public class PositionFSM {

    public enum States {
        ZONE_1(30), // TODO: check does pitch at 22.5 equal like almost 360?
        ZONE_2(26),
        ZONE_3(24),
        ZONE_4(15),
        ZONE_5(16),

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

    public enum Sensor{
        LIMELIGHT,
        PINPOINT
    }

    private States state;
    public static Sensor sensor;
    private LimelightCamera limelightCamera;
    private Pinpoint pinpoint;
    private InterpLUT velocityMapLL;
    private InterpLUT velocityMapPP;



    private double defaultFlywheelVelocity = 2500;
    private double flywheelTargetVelocityRPM;
    private double pitchTargetAngle;
    private double turretError;

    private double LIMELIGHT_FORWARD_OFFSET = 0; // TODO: x: 60.05 mm, y: 53.845 mm, distance: 80.656 mm
    private double PINPOINT_OFFSET = 0;

    private double threshold1LL = 1.5, threshold2LL = 2, threshold3LL = 2.5, threshold4LL = 3;

    private double threshold1PP = 1.4, threshold2PP = 2, threshold3PP = 2.5, threshold4PP = 3;

    private double SENSOR_CHOICE_THRESHOLD = 2;
    private double RELOCALIZATION_TRHESHOLD = 0.1;
    private boolean rumbleNotification = false;
    public static double CAMERA_DISTANCE_FROM_CENTER = 0.1;

    private DoubleSupplier turretAngleProvider;

    private Telemetry telemetry;

    private RobotSettings robotSettings;

    public static double flywheelRPM = 3500;

    private Pose2D newPoseRelocal = null;

    private boolean lastRightBumper = false;

    private Logger logger;


    public PositionFSM(HWMap hwMap, Telemetry telemetry, Pinpoint pinpoint, DoubleSupplier turretAngleProvider, RobotSettings robotSettings, Logger logger) {
        this.logger = logger;
        limelightCamera = new LimelightCamera(hwMap.getLimelight(), telemetry, robotSettings);
        this.pinpoint = pinpoint;
        state = States.NO_VALID_TARGET;
        sensor = Sensor.PINPOINT;
        this.turretAngleProvider = turretAngleProvider;
        createVelocityMap();
        this.telemetry = telemetry;
        this.robotSettings = robotSettings;
    }

    public void updateState(boolean rightBumper2) {
        if(rightBumper2 && !lastRightBumper) {
            resetOdo();
        }

        lastRightBumper = rightBumper2;
        limelightCamera.update();
        chooseSensor();

        if(limelightCamera.hasTarget() || pinpoint.pinpointReady()) {
            if(sensor == Sensor.LIMELIGHT) {
                if(limelightCamera.getFlatDistance() >= threshold4LL) {
                    state = States.ZONE_5;
                }
                else if (limelightCamera.getFlatDistance() >= threshold3LL) {
                    state = States.ZONE_4;
                }
                else if (limelightCamera.getFlatDistance() >= threshold2LL) {
                    state = States.ZONE_3;
                } else if (limelightCamera.getFlatDistance() >= threshold1LL) {
                    state = States.ZONE_2;
                } else {
                    state = States.ZONE_1;
                }
                findFlywheelTargetVelocity(limelightCamera.getFlatDistance());
                findPitchTargetAngle();
                findTurretError(limelightCamera.getTy());
            }
            else if(sensor == Sensor.PINPOINT) {
                if (pinpoint.pinpointReady()) {
                    if(pinpoint.getGoalDistance() >= threshold4PP) {
                        state = States.ZONE_5;
                    }
                    else if (pinpoint.getGoalDistance() >= threshold3PP) {
                        state = States.ZONE_4;
                    }
                    else if (pinpoint.getGoalDistance() >= threshold2PP) {
                        state = States.ZONE_3;
                    } else if (pinpoint.getGoalDistance() >= threshold1PP) {
                        state = States.ZONE_2;
                    } else {
                        state = States.ZONE_1;
                    }
                    findFlywheelTargetVelocity(pinpoint.getGoalDistance());
                    findPitchTargetAngle();
                    findTurretError(pinpoint.getHeadingErrorTrig());
                }
                else {
                    state = States.NO_VALID_TARGET;
                }
            }
        }
        else {
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

        // FOR Limelight
        velocityMapLL = new InterpLUT();

        // distance (m) , velocity (rpm)

        velocityMapLL.add(0.5,2500);
        velocityMapLL.add(1.35, 2550);
        velocityMapLL.add(1.58, 2625);
        velocityMapLL.add(2.21, 3050);
        velocityMapLL.add(2.85, 3400);
        velocityMapLL.add(3.25, 3700);
        velocityMapLL.add(3.5,4000);
        velocityMapLL.createLUT();


        // FOR PINPOINT

        velocityMapPP = new InterpLUT();


        velocityMapPP.add(0.5,2500);
        velocityMapPP.add(1.19, 2600);
        velocityMapPP.add(1.44, 2600);
        velocityMapPP.add(2.04, 2900);
        velocityMapPP.add(2.63, 3250);
        velocityMapPP.add(3.03, 3700);
        velocityMapPP.add(3.5,4000);
        velocityMapPP.createLUT();

    }

    public void findFlywheelTargetVelocity(double distance_m) {
        if(distance_m <= 0.5 || distance_m >= 3.5 || Double.isNaN(distance_m)) {
            flywheelTargetVelocityRPM = defaultFlywheelVelocity;
        }
        else {
            if(sensor == Sensor.LIMELIGHT) {
                flywheelTargetVelocityRPM = velocityMapLL.get(distance_m + LIMELIGHT_FORWARD_OFFSET);
            }
            else {
                flywheelTargetVelocityRPM = flywheelRPM;
               // flywheelTargetVelocityRPM = velocityMapPP.get(distance_m + PINPOINT_OFFSET);
            }
        }

    }

    public void findPitchTargetAngle() {
        if(state == States.NO_VALID_TARGET) {
            return;
        }
        pitchTargetAngle = state.getTargetAngle();
    }

    public void findTurretError(double error) {
        if(state == States.NO_VALID_TARGET || Double.isNaN(error)) {
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

    private void chooseSensor() {
        if(robotSettings.distanceMethod.equals(RobotSettings.DistanceMethod.LIMELIGHT_ONLY)) {
                sensor = Sensor.LIMELIGHT;
        }
        else if(robotSettings.distanceMethod.equals(RobotSettings.DistanceMethod.PINPOINT_ONLY)) {
            sensor = Sensor.PINPOINT;
        }
        else if(robotSettings.distanceMethod.equals(RobotSettings.DistanceMethod.LIMELIGHT_AND_PINPOINT)) {
             if (!pinpoint.pinpointReady()) {
                sensor = Sensor.LIMELIGHT;
            }
            else {
                sensor = Sensor.PINPOINT;
            }
            if(Math.abs(pinpoint.getGoalDistance() - limelightCamera.getFlatDistance()) > 0.1 && pinpoint.pinpointReady() && limelightCamera.hasTarget()) {
                rumbleNotification = true;
            }
            else {
                rumbleNotification = false;
            }
        }

    }

    public void relocalize() {
        //TODO: add 20 degree pitch and add 90 degree roll
        double turretHeading = -turretAngleProvider.getAsDouble();
        double CamOffsetHeadingFromTurret = 180+48.064;
        double robotHeading = pinpoint.getHeading();
        double cameraAbsoluteHeading = robotHeading + turretHeading + CamOffsetHeadingFromTurret;

        double camX = limelightCamera.getxField();
        double camY = limelightCamera.getyField();
        double distanceCamToTurretCenter = 0.07207114;

        double distanceTurretCenterToRobotCenter = 0.04;

        double xTurret =  camX - (distanceCamToTurretCenter*(Math.cos(Math.toRadians(cameraAbsoluteHeading))));
        double yTurret = camY - (distanceCamToTurretCenter*(Math.sin(Math.toRadians(cameraAbsoluteHeading))));

        double xRobot = xTurret - (distanceTurretCenterToRobotCenter*(Math.cos(Math.toRadians(robotHeading + 180))));
        double yRobot = yTurret - (distanceTurretCenterToRobotCenter*(Math.sin(Math.toRadians(robotHeading + 180))));

        Pose2D newPos = new Pose2D(DistanceUnit.METER, xRobot,yRobot, AngleUnit.DEGREES, pinpoint.getHeading());
        pinpoint.setPosition(newPos);

        telemetry.addLine("--- RELOCALIZATION ---");
        telemetry.addData("Cam absolute Heading", cameraAbsoluteHeading);
        telemetry.addData("Robot Heading", robotHeading);
        telemetry.addData("Turret Heading", turretHeading);
        telemetry.addData("Turret X", xTurret);
        telemetry.addData("Turret Y", yTurret);
        telemetry.addData("Cam X", camX);
        telemetry.addData("Cam Y", camY);
        telemetry.addData("Robot X", xRobot);
        telemetry.addData("Robot Y", yRobot);
    }


    public void log() {
        logger.log("----------POSITION FSM LOG----------", "", Logger.LogLevels.PRODUCTION);
        logger.log("position FSM state", state, Logger.LogLevels.DEBUG);
        logger.log("Current shooting Sensor", sensor, Logger.LogLevels.PRODUCTION);
        logger.log("Flywheel Target", flywheelTargetVelocityRPM, Logger.LogLevels.DEBUG);
        logger.log("Pitch Target", pitchTargetAngle, Logger.LogLevels.DEBUG);
        logger.log("Turret Error", turretError, Logger.LogLevels.DEBUG);


        logger.log("----------LIMELIGHT LOG----------", "", Logger.LogLevels.PRODUCTION);
        logger.log("X", limelightCamera.getX(), Logger.LogLevels.DEBUG);
        logger.log("Y", limelightCamera.getY(), Logger.LogLevels.DEBUG);
        logger.log("Z", limelightCamera.getZ(), Logger.LogLevels.DEBUG);
        logger.log("Flat Distance", limelightCamera.getFlatDistance(), Logger.LogLevels.DEBUG);
        logger.log("tx", limelightCamera.getTx(), Logger.LogLevels.DEBUG);
        logger.log("ty", limelightCamera.getTy(), Logger.LogLevels.DEBUG);
        logger.log("Has target", limelightCamera.hasTarget(), Logger.LogLevels.PRODUCTION);
        logger.log("----------LIMELIGHT LOG----------", "", Logger.LogLevels.PRODUCTION);

        logger.log("----------PINPOINT LOG----------", "", Logger.LogLevels.PRODUCTION);
        logger.log("Goal Distance", pinpoint.getGoalDistance(), Logger.LogLevels.PRODUCTION);
        logger.log("pinpoint heading error", pinpoint.getHeadingErrorTrig(), Logger.LogLevels.PRODUCTION);
        logger.log("pinpoint ready", pinpoint.pinpointReady(), Logger.LogLevels.PRODUCTION);
        logger.log("Pinpoint x", pinpoint.getX(), Logger.LogLevels.PRODUCTION);
        logger.log("Pinpoint y", pinpoint.getY(), Logger.LogLevels.PRODUCTION);
        logger.log("----------PINPOINT LOG----------", "", Logger.LogLevels.PRODUCTION);

        logger.log("----------POSITION FSM LOG----------", "", Logger.LogLevels.PRODUCTION);

        /*
        if(newPoseRelocal != null) {
            telemetry.addData("Relocalization new Pos x", newPoseRelocal.getX(DistanceUnit.METER));
            telemetry.addData("Relocalization new Pos y", newPoseRelocal.getY(DistanceUnit.METER));
        }*/


    }

    public Pose2D getRobotPos() {
        //if(limelightCamera.hasTarget()) {
            newPoseRelocal = new Pose2D(DistanceUnit.METER, limelightCamera.getxField(), limelightCamera.getyField(), AngleUnit.DEGREES, pinpoint.getHeading());
        //}
        return newPoseRelocal;
    }

    public void resetOdo() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.METER,1.2,0,AngleUnit.DEGREES,pinpoint.getHeading()));
    }

}
