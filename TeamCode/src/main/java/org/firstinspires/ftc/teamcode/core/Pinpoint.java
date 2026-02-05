package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.pedropathing.localization.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class Pinpoint {  // TODO: add junit

    GoBildaPinpointDriver odo;
    Pose2D pos;
    public double x, y, heading;
    public static double Xoffset, Yoffset;



    public Pinpoint(HWMap hwMap, RobotSettings robotSettings) {
        odo = hwMap.getOdo();
        Xoffset = -132.5; Yoffset = 14.075;

        odo.setOffsets(Xoffset, Yoffset, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


       // odo.resetPosAndIMU();

       // Pose2D pose2D = new Pose2D(DistanceUnit.METER, 0, 0.0, AngleUnit.DEGREES, 0.0);
    //  odo.setPosition(robotSettings.startPosState.getPose2D());


        odo.setPosition(new Pose2D(DistanceUnit.INCH,86.720, 137.685,AngleUnit.DEGREES,0));
        update();
    }

    public boolean pinpointReady() {
        return odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public void update() {
        odo.update();
        pos = odo.getPosition();
        x = pos.getX(DistanceUnit.METER);
        y= pos.getY(DistanceUnit.METER);
        heading = pos.getHeading(AngleUnit.DEGREES);
    }

    public void updateHeadingOnly() {
        odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        heading = Math.toDegrees(odo.getHeading(AngleUnit.DEGREES));
    }

    public Pose2D getPos() {
        return pos;
    }

    public double getHeading() {
        return heading;
    }


    public double getX() {
        return x;
    }


    public double getY() {
        return y;
    }

    public void resetPosAndIMU() {
        odo.resetPosAndIMU();
    }

    public double getGoalDistance() {
        return Math.sqrt(Math.pow((RobotSettings.alliance.getGoalPos().getX(DistanceUnit.METER) - x), 2) + Math.pow((RobotSettings.alliance.getGoalPos().getY(DistanceUnit.METER) - y), 2));
    }

    public void setPosition(Pose2D pose2D) {
        odo.setPosition(pose2D);
        odo.update();
    }
/*
public double getGoalHeading() {
        double error;
        if(MainAuto.ALLIANCE.equals("RED")) {
            error = RED_GOAL_POS.getHeading(AngleUnit.DEGREES) - heading;
        }
        else {
            error = BLUE_GOAL_POS.getHeading(AngleUnit.DEGREES) - heading;
        }
        if (Math.signum(error) != Math.signum(getHeadingErrorTrig())) {
            error = -error;
        }
        return error;
    }*/


    public double getHeadingErrorTrig() {
        double targetAngle;
        targetAngle = Math.toDegrees(Math.atan2((RobotSettings.alliance.getGoalPos().getY(DistanceUnit.METER) - y), (RobotSettings.alliance.getGoalPos().getX(DistanceUnit.METER) - x)));

        double error = targetAngle - heading;

        if(error <= -180) {
            error += 360;
        }
        else if (error >= 180) {
            error -= 360;
        }
 error = -error;
        error = 360 - error;

        return error;
    }

}
