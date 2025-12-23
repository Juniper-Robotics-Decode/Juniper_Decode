package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class RobotSettings {
    public enum Alliance{
        RED(new Pose2D(DistanceUnit.METER, -1.482, 1.413, AngleUnit.DEGREES, 136.4)),
        BLUE (new Pose2D(DistanceUnit.METER, -1.482, -1.413, AngleUnit.DEGREES, 223.6));

        private Pose2D pose2D;

        Alliance(Pose2D pos) {
            this.pose2D = pos;
        }

        public Pose2D getGoalPos() {
            return pose2D;
        }
    }

    public enum DistanceMethod{
        LIMELIGHT_ONLY,
        PINPOINT_ONLY,
        LIMELIGHT_AND_PINPOINT
    }
    public enum StartPos{
        CLOSE_RED (new Pose2D(DistanceUnit.METER, -1.64, -0.1625, AngleUnit.DEGREES, 0.0)),
        FAR_RED (new Pose2D(DistanceUnit.METER, 0.0, 0.0, AngleUnit.DEGREES, 0.0)),
        CLOSE_BLUE (new Pose2D(DistanceUnit.METER, 0.0, 0.0, AngleUnit.DEGREES, 0.0)),
        FAR_BLUE((new Pose2D(DistanceUnit.METER, 0.0, 0.0, AngleUnit.DEGREES, 0.0)));

        private Pose2D pose2D;

        StartPos(Pose2D pos) {
            this.pose2D = pos;
        }

        public Pose2D getPose2D() {
            return pose2D;
        }
    }

    public static Alliance alliance;
    public static DistanceMethod distanceMethod;
    public static StartPos startPosState;


    public RobotSettings () {
        alliance = Alliance.RED;
        distanceMethod = DistanceMethod.PINPOINT_ONLY;
        startPosState = StartPos.CLOSE_RED;
    }

}
