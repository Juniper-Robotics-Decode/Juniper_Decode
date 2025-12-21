package org.firstinspires.ftc.teamcode.shooter.wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.MainAuto;
import org.firstinspires.ftc.teamcode.core.RobotSettings;

@Config
public class Pinpoint {  // TODO: add junit

    GoBildaPinpointDriver odo;
    Pose2D pos;
    public static double x, y, heading;
    public static double Xoffset, Yoffset;


    public static Pose2D RED_GOAL_POS = new Pose2D(DistanceUnit.METER, -1.482, -1.413, AngleUnit.DEGREES, 136.4);
    public static Pose2D BLUE_GOAL_POS = new Pose2D(DistanceUnit.METER, -1.482, 1.413, AngleUnit.DEGREES, 0.0); // TODO: add right angle


    public Pinpoint(HWMap hwMap, RobotSettings robotSettings) {
        odo = hwMap.getOdo();
        Xoffset = -132.5; Yoffset = 14.075;
        odo.setOffsets(Xoffset, Yoffset);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

       // odo.resetPosAndIMU();

        Pose2D pose2D = new Pose2D(DistanceUnit.METER, -1.64, 0.27, AngleUnit.DEGREES, 0.0);
        odo.setPosition(pose2D);
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
        heading = -pos.getHeading(AngleUnit.DEGREES);
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

    public double getGoalDistance() {
        if(MainAuto.ALLIANCE.equals("RED")) {
            return Math.sqrt(Math.pow((RED_GOAL_POS.getX(DistanceUnit.METER) - x), 2) + Math.pow((RED_GOAL_POS.getY(DistanceUnit.METER) - y), 2));
        }
        else {
            return Math.sqrt(Math.pow((BLUE_GOAL_POS.getX(DistanceUnit.METER) - x),2) + Math.pow((BLUE_GOAL_POS.getY(DistanceUnit.METER) - y),2));
        }
    }

    /*public double getGoalHeading() {
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

    }
*/
    public double getHeadingErrorTrig() {
        double targetAngle;
        if(MainAuto.ALLIANCE.equals("RED")) {
            targetAngle = Math.toDegrees(Math.atan2((RED_GOAL_POS.getY(DistanceUnit.METER) - y), (RED_GOAL_POS.getX(DistanceUnit.METER) - x)));
        }
        else {
            targetAngle = Math.toDegrees(Math.atan2((BLUE_GOAL_POS.getY(DistanceUnit.METER) - y), (BLUE_GOAL_POS.getX(DistanceUnit.METER) - x)));
        }
        double error = targetAngle - heading;
        if(error <= -180) {
            error += 360;
        }
        else if (error >= 180) {
            error -= 360;
        }
        return error;
    }

}
