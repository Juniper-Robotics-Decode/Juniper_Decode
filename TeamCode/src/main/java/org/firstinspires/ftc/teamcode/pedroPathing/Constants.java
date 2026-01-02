package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.drivetrains.Swerve;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-117.118)
            .lateralZeroPowerAcceleration(-117.118)
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.005 , 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.003, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0 , 0.003, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.63, 0, 0.035, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(1, 0, 0.00001, 0.6, 0.13))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.000005, 0.6, 0.13))
            .centripetalScaling(0.005)



//            .headingPIDFSwitch(Math.toRadians(15))
//            .translationalPIDFSwitch(5)
            .mass(7.801789); //TODO: actually weigh the robot, in kg





    public static SwerveConstants driveConstants = new SwerveConstants()
            .velocity(48) // in per sec
//            .maxPower(.75)
            .staticFrictionCoefficient(0.02);

    private static CoaxialPod leftFront(HardwareMap hardwareMap) {
        return new CoaxialPod(hardwareMap,"FLM","FLS","FLE",
                new PIDFCoefficients(0.6*(Math.PI/180.0), 0.0, 0.002*(Math.PI/180.0),0),
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
                Math.toDegrees(2.7),
                new Pose(4.963582677,4.96063),
                0,3.3,
                false);
    }

    private static CoaxialPod rightFront(HardwareMap hardwareMap) {
        return new CoaxialPod(hardwareMap,"FRM","FRS","FRE",
                new PIDFCoefficients(0.6*(Math.PI/180.0), 0.0, 0.002*(Math.PI/180.0),0),
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
                Math.toDegrees(3.5),
                new Pose(4.963582677,-4.96063),
                0,3.3,
                false);
    }

    private static CoaxialPod leftBack(HardwareMap hardwareMap) {
        return new CoaxialPod(hardwareMap,"BLM","BLS","BLE",
                new PIDFCoefficients(0.6*(Math.PI/180.0), 0.0, 0.002*(Math.PI/180.0),0),
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
                Math.toDegrees(3.4),
                new Pose(-4.963582677,4.96063),
                0,3.3,
                false);
    }

    private static CoaxialPod rightBack(HardwareMap hardwareMap) {
        return new CoaxialPod(hardwareMap,"BRM","BRS","BRE",
                new PIDFCoefficients(0.6*(Math.PI/180.0), 0.0, 0.002*(Math.PI/180.0),0),
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
                Math.toDegrees(1.17),
                new Pose(-4.963582677,-4.96063),
                0,3.3,
                false);
    }


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5.2165354) // -132.5 mm
            .strafePodX(0.554133858) // 14.074 mm
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .swerveDrivetrain(driveConstants, leftFront(hardwareMap),rightFront(hardwareMap),leftBack(hardwareMap),rightBack(hardwareMap))
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
