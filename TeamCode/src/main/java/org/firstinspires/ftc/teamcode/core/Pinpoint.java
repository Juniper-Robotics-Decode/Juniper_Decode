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
    private RobotSettings robotSettings;


    // TODO: need to convert coordinates
    // coordinates for Pinpoint derived from Pedro
    // x = -y
    // y = x
    // theta = thetaPedro + 90

    public Pinpoint(HWMap hwMap, RobotSettings robotSettings, boolean isAuto) {
        odo = hwMap.getOdo();
        this.robotSettings = robotSettings;
        Xoffset = -132.5; Yoffset = 14.075;

        odo.setOffsets(Xoffset, Yoffset, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections( GoBildaPinpointDriver.EncoderDirection.FORWARD,  GoBildaPinpointDriver.EncoderDirection.FORWARD);


       // odo.resetPosAndIMU();

       // Pose2D pose2D = new Pose2D(DistanceUnit.METER, 0, 0.0, AngleUnit.DEGREES, 0.0);
      //  if (isAuto) {
         //   odo.setPosition(robotSettings.startPosState.getPose2D());
       // }

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
        x = pos.getX(DistanceUnit.INCH);
        y= pos.getY(DistanceUnit.INCH);
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

    public void resetIMU() {
        odo.setPosition(new Pose2D(DistanceUnit.METER,x,y,AngleUnit.DEGREES,0));
    }

    public void resetPos() {
        odo.setPosition(new Pose2D(DistanceUnit.METER,1.2,0,AngleUnit.DEGREES,heading));
    }

    public double getGoalDistance() {
        return Math.sqrt(Math.pow((138 - x), 2) + Math.pow((138 - y), 2));
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
        targetAngle = Math.toDegrees(Math.atan2((138 - y), (138 - x)));

        double error = targetAngle - heading;

        if(error <= -180) {
            error += 360;
        }
        else if (error >= 180) {
            error -= 360;
        }

        /*error = -error;
        error = 360 - error;
*/
        return error;
    }

}
