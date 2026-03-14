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
        ZONE_0(5, -0.1),
        ZONE_1(15, -0.1), // TODO: check does pitch at 22.5 equal like almost 360? -.1
        ZONE_2(15, -0.1), // -.015
        ZONE_3(15, -0.1), // -.15
        ZONE_4(20,-1.4), //-1.4
        ZONE_5(18,-2.1), // -1.5


        NO_VALID_TARGET;

        private double targetAngle;

        private double pitchCompensation;

        States(double angle, double pitchComp) {
            this.targetAngle = angle;
            pitchCompensation = pitchComp;
        }

        States() {
            this.targetAngle = 0;
            this.pitchCompensation = 0;
            // doesn't matter if 0 - never used
        }

        public double getTargetAngle() {
            return targetAngle;
        }

        public double getPitchCompensation() {
            return pitchCompensation;
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

    double X = 0;
    double Y = 0;

    private double defaultFlywheelVelocity = 2500;
    private double flywheelTargetVelocityRPM;
    private double pitchTargetAngle;
    private double pitchCompensation;
    private double turretError;

    private double LIMELIGHT_FORWARD_OFFSET = 0; // TODO: x: 60.05 mm, y: 53.845 mm, distance: 80.656 mm
    private double PINPOINT_OFFSET = 0;


    private double threshold1LL = 1.5, threshold2LL = 2, threshold3LL = 2.5, threshold4LL = 3;

    private double threshold1PP = 46.85039, threshold2PP = 65, threshold3PP = 88.955, threshold4PP = 115;

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


    public PositionFSM(HWMap hwMap, Telemetry telemetry, DoubleSupplier turretAngleProvider, RobotSettings robotSettings, Logger logger, Pinpoint pinpoint) {
        this.logger = logger;
        limelightCamera = new LimelightCamera(hwMap.getLimelight(), telemetry, robotSettings);
        this.pinpoint = pinpoint;
        state = States.NO_VALID_TARGET;
        sensor = Sensor.PINPOINT;
        this.turretAngleProvider = turretAngleProvider;
        createVelocityMap();
        this.telemetry = telemetry;
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        this.robotSettings = robotSettings;
    }

    public void updateState() {
      //  chooseSensor();

        sensor = Sensor.PINPOINT;


            if (sensor == Sensor.LIMELIGHT) {
                limelightCamera.update();
                if(limelightCamera.hasTarget()) {
                    if (limelightCamera.getGoalDistance() >= threshold4PP) {
                        state = States.ZONE_5;
                    } else if (limelightCamera.getGoalDistance() >= threshold3PP) {
                        state = States.ZONE_4;
                    } else if (limelightCamera.getGoalDistance() >= threshold2PP) {
                        state = States.ZONE_2;
                    } else if (limelightCamera.getGoalDistance() >= threshold1PP) {
                        state = States.ZONE_1;
                    } else {
                        state = States.ZONE_0;
                    }
                    findFlywheelTargetVelocity(limelightCamera.getGoalDistance());
                    findPitchTargetAngle();
                    findTurretError(limelightCamera.getTx());
                }
                else {
                    state = States.NO_VALID_TARGET;
                }
            }
            else if (sensor == Sensor.PINPOINT) {
              //  limelightCamera.update();
                pinpoint.update();
              //  if(pinpoint.pinpointReady()) {
                    if (pinpoint.getGoalDistance() >= threshold4PP) {
                        state = States.ZONE_5;
                    } else if (pinpoint.getGoalDistance() >= threshold3PP) {
                        state = States.ZONE_4;
                    } else if (pinpoint.getGoalDistance() >= threshold2PP) {
                        state = States.ZONE_2;
                    } else if (pinpoint.getGoalDistance() >= threshold1PP) {
                        state = States.ZONE_1;
                    } else {
                        state = States.ZONE_0;
                    }
                    findFlywheelTargetVelocity(pinpoint.getGoalDistance());
                    findPitchTargetAngle();
                    findTurretError(pinpoint.getHeadingErrorTrig());
              //  }
                /*else {
                    state = States.NO_VALID_TARGET;
                }*/
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
        velocityMapLL.add(0.861, 2900);
        velocityMapLL.add(1.35, 3000);
        velocityMapLL.add(2.21, 3100);
        velocityMapLL.add(2.85, 3250);
        velocityMapLL.add(3.25, 3400);
        velocityMapLL.add(3.5,3650);
        velocityMapLL.createLUT();


        // FOR PINPOINT

        velocityMapPP = new InterpLUT();

        velocityMapPP.add(19.685,2450);
        velocityMapPP.add(33.898, 2450);
        velocityMapPP.add(46.85039, 2800);
        velocityMapPP.add(56.85039, 2800);
        velocityMapPP.add(80.31496, 2950);
        velocityMapPP.add(103.5433, 3200);
        velocityMapPP.add(119.2913, 3200);
        velocityMapPP.add(147.14,3425);
        velocityMapPP.add(147.2441,3650);
        velocityMapPP.createLUT();

    }

    public void findFlywheelTargetVelocity(double distance_m) {
        if(distance_m <= 20 || distance_m >= 147 || Double.isNaN(distance_m)) {
            flywheelTargetVelocityRPM = defaultFlywheelVelocity;
        }
        else {
            if(sensor == Sensor.LIMELIGHT) {
                flywheelTargetVelocityRPM = velocityMapLL.get(distance_m + LIMELIGHT_FORWARD_OFFSET);
            }
            else {
                flywheelTargetVelocityRPM = velocityMapPP.get(distance_m + PINPOINT_OFFSET);
            }
        }

    }

    public void findPitchTargetAngle() {
        if(state == States.NO_VALID_TARGET) {
            return;
        }
        pitchTargetAngle = state.getTargetAngle();
        pitchCompensation = state.getPitchCompensation();
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

    public double getPitchCompensation() {
        return pitchCompensation;
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
            /* if (!pinpoint.pinpointReady()) {
                sensor = Sensor.LIMELIGHT;
            }
            else {
                sensor = Sensor.PINPOINT;
            }*/
//            if(Math.abs(pinpoint.getGoalDistance() - limelightCamera.getFlatDistance()) > 0.1 && pinpoint.pinpointReady() && limelightCamera.hasTarget()) {
//                rumbleNotification = true;
//            }
//            else {
//                rumbleNotification = false;
//            }
        }

    }

//    public void relocalize() {
//        //TODO: add 20 degree pitch and add 90 degree roll
//        double turretHeading = -turretAngleProvider.getAsDouble();
//        double CamOffsetHeadingFromTurret = 180+48.064;
//        double robotHeading = pinpoint.getHeading();
//        double cameraAbsoluteHeading = robotHeading + turretHeading + CamOffsetHeadingFromTurret;
//
//        double camX = limelightCamera.getxField();
//        double camY = limelightCamera.getyField();
//        double distanceCamToTurretCenter = 0.07207114;
//
//        double distanceTurretCenterToRobotCenter = 0.04;
//
//        double xTurret =  camX - (distanceCamToTurretCenter*(Math.cos(Math.toRadians(cameraAbsoluteHeading))));
//        double yTurret = camY - (distanceCamToTurretCenter*(Math.sin(Math.toRadians(cameraAbsoluteHeading))));
//
//        double xRobot = xTurret - (distanceTurretCenterToRobotCenter*(Math.cos(Math.toRadians(robotHeading + 180))));
//        double yRobot = yTurret - (distanceTurretCenterToRobotCenter*(Math.sin(Math.toRadians(robotHeading + 180))));
//
//        Pose2D newPos = new Pose2D(DistanceUnit.METER, xRobot,yRobot, AngleUnit.DEGREES, pinpoint.getHeading());
//        pinpoint.setPosition(newPos);
//
//        telemetry.addLine("--- RELOCALIZATION ---");
//        telemetry.addData("Cam absolute Heading", cameraAbsoluteHeading);
//        telemetry.addData("Robot Heading", robotHeading);
//        telemetry.addData("Turret Heading", turretHeading);
//        telemetry.addData("Turret X", xTurret);
//        telemetry.addData("Turret Y", yTurret);
//        telemetry.addData("Cam X", camX);
//        telemetry.addData("Cam Y", camY);
//        telemetry.addData("Robot X", xRobot);
//        telemetry.addData("Robot Y", yRobot);
//    }


    public void log() {
        logger.log("<font color='yellow'>----------POSITION FSM LOG----------</font>", "", Logger.LogLevels.PRODUCTION);
        logger.log("position FSM state", state, Logger.LogLevels.DEBUG);
        logger.log("<font color='orange'>Current shooting Sensor</font>", sensor, Logger.LogLevels.PRODUCTION);
        logger.log("Flywheel Target", flywheelTargetVelocityRPM, Logger.LogLevels.DEBUG);
        logger.log("Pitch Target", pitchTargetAngle, Logger.LogLevels.DEBUG);
        logger.log("Pitch compensation", pitchCompensation, Logger.LogLevels.DEBUG);
        logger.log("Turret Error", turretError, Logger.LogLevels.DEBUG);
    }
    public void logLL() {
        logger.log("<font color='yellow'>----------LIMELIGHT LOG----------</font>", "", Logger.LogLevels.PRODUCTION);
        logger.log("X", limelightCamera.getX(), Logger.LogLevels.DEBUG);
        logger.log("Y", limelightCamera.getY(), Logger.LogLevels.DEBUG);
        logger.log("Z", limelightCamera.getZ(), Logger.LogLevels.DEBUG);
        logger.log("Flat Distance", limelightCamera.getFlatDistance(), Logger.LogLevels.DEBUG);
        logger.log("tx", limelightCamera.getTx(), Logger.LogLevels.DEBUG);
        logger.log("ty", limelightCamera.getTy(), Logger.LogLevels.DEBUG);
        logger.log("Has target", limelightCamera.hasTarget(), Logger.LogLevels.DEBUG);
        logger.log("<b><font color='pink'>Limelight X</b></font>", limelightCamera.getxField(), Logger.LogLevels.PRODUCTION);
        logger.log("<b><font color='pink'>Limelight Y</b></font>", limelightCamera.getyField(), Logger.LogLevels.PRODUCTION);
    }
    public void logPP() {
        logger.log("<font color='yellow'>----------PINPOINT LOG----------</font>", "", Logger.LogLevels.PRODUCTION);
        logger.log("Goal Distance", pinpoint.getGoalDistance(), Logger.LogLevels.DEBUG);
        logger.log("pinpoint heading error", pinpoint.getHeadingErrorTrig(), Logger.LogLevels.DEBUG);
        logger.log("pinpoint ready", pinpoint.pinpointReady(), Logger.LogLevels.DEBUG);
        logger.log("<b><font color='purple'>Pinpoint X</b></font>", pinpoint.getX(), Logger.LogLevels.PRODUCTION);
        logger.log("<b><font color='purple'>Pinpoint Y</b></font>", pinpoint.getY(), Logger.LogLevels.PRODUCTION);
        logger.log("<b><font color='purple'>Pinpoint Heading</b></font>", pinpoint.getHeading(), Logger.LogLevels.PRODUCTION);
    }
    /*
        if(newPoseRelocal != null) {
            telemetry.addData("Relocalization new Pos x", newPoseRelocal.getX(DistanceUnit.METER));
            telemetry.addData("Relocalization new Pos y", newPoseRelocal.getY(DistanceUnit.METER));
        }*/

    public Pose2D getRobotPos() {
        //if(limelightCamera.hasTarget()) {
            newPoseRelocal = new Pose2D(DistanceUnit.METER, limelightCamera.getxField(), limelightCamera.getyField(), AngleUnit.DEGREES, pinpoint.getHeading());
        //}
        return newPoseRelocal;
    }

    public Pose2D relocalize(){
        limelightCamera.update();
        X = (limelightCamera.getyField()/DistanceUnit.mPerInch) + 72;
        Y = -(limelightCamera.getxField()/DistanceUnit.mPerInch) + 72;
       // pinpoint.update();
        return new Pose2D(DistanceUnit.INCH,X,Y, AngleUnit.DEGREES, pinpoint.getHeading());
    }

}
