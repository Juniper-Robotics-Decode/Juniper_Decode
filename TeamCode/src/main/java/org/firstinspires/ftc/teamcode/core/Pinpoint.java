package org.firstinspires.ftc.teamcode.core;

import com.pedropathing.localization.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Pinpoint {

    GoBildaPinpointDriver odo;
    Pose2D pos;
    public static double x, y, heading;
    public static double Xoffset, Yoffset;


    public static Pose2D RED_GOAL_POS = new Pose2D(DistanceUnit.METER, -1.482, 1.413, AngleUnit.DEGREES, 0.0);
    public static Pose2D BLUE_GOAL_POS = new Pose2D(DistanceUnit.METER, -1.482, -1.413, AngleUnit.DEGREES, 0.0);


    public Pinpoint(HWMap hwMap) {
        odo = hwMap.getOdo();
        Xoffset = 14; Yoffset = 4;
        odo.setOffsets(Xoffset, Yoffset);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();
        pos = new Pose2D(DistanceUnit.METER, 0,0,AngleUnit.DEGREES, 0);
        odo.setPosition(pos);
    }

    public boolean pinpointReady() {
        return odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public void update() {
        odo.update();
        pos = odo.getPosition();
        x = odo.getPosX();
        y= odo.getPosY();
        heading = Math.toDegrees(odo.getHeading());
    }

    public void updateHeadingOnly() {
        odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        heading = Math.toDegrees(odo.getHeading());
    }

    public Pose2D getPos() {
        return pos;
    }

    public static double getHeading() {
        return heading;
    }


    public static double getX() {
        return x;
    }


    public static double getY() {
        return y;
    }

    public void resetPosAndIMU() {
        odo.resetPosAndIMU();
    }

    public double getGoalDistance() {
        if(MainAuto.ALLIANCE.equals("RED")) {
            return Math.sqrt(Math.pow((RED_GOAL_POS.getX(DistanceUnit.METER) - x), 2) + Math.pow((RED_GOAL_POS.getY(DistanceUnit.METER) - y), 2));
        }
        else {
            return Math.sqrt(Math.pow((BLUE_GOAL_POS.getX(DistanceUnit.METER) - x),2) + Math.pow((BLUE_GOAL_POS.getY(DistanceUnit.METER) - y),2));
        }
    }

    public double getHeadingError() {
        double targetAngle;
        if(MainAuto.ALLIANCE.equals("RED")) {
            targetAngle = Math.toDegrees(Math.atan2((RED_GOAL_POS.getY(DistanceUnit.METER) - y), (RED_GOAL_POS.getX(DistanceUnit.METER) - x)));
        }
        else {
            targetAngle = Math.toDegrees(Math.atan2((BLUE_GOAL_POS.getY(DistanceUnit.METER) - y), (BLUE_GOAL_POS.getX(DistanceUnit.METER) - x)));
        }
        return targetAngle - heading;
    }


}
