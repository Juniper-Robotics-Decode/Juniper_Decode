package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.Swerve;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-138.72)
            .lateralZeroPowerAcceleration(-138.72)
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.01, 0, 0.005 , 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.003, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.25, 0 , 0.003, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.63, 0, 0.035, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0035, 0, 0.00001, 0.6, 0.13))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.000005, 0.6, 0.13))

//            .headingPIDFSwitch(Math.toRadians(15))
//            .translationalPIDFSwitch(5)
            .mass(9.07185); //TODO: actually weigh the robot, in kg

    //top left = 1191g
    //top right = 1400g
    //bottom left = 1376g
    //bottom right = 1395g
    //total = 5362g

    public static SwerveConstants driveConstants = new SwerveConstants()
            .velocity(77.45)
//            .maxPower(.75)
            .useBrakeModeInTeleOp(true)
            .leftFrontServoName("FLS")
            .leftFrontEncoderName("FLE")
            .leftFrontMotorName("FLM")
            .rightFrontServoName("FRS")
            .rightFrontEncoderName("FRE")
            .rightFrontMotorName("FRM")
            .leftRearServoName("BLS")
            .leftRearEncoderName("BLE")
            .leftRearMotorName("BLM")
            .rightRearServoName("BRS")
            .rightRearEncoderName("BRE")
            .rightRearMotorName("BRM")
            .leftFrontTurnPID(new PIDFCoefficients( 0.4, 0.0, 0.002, 0.02))
            .rightFrontTurnPID(new PIDFCoefficients( 0.4, 0.0, 0.002, 0.02))
            .leftRearTurnPID(new PIDFCoefficients( 0.4, 0.0, 0.002, 0.02))
            .rightRearTurnPID(new PIDFCoefficients( 0.4, 0.0, 0.002, 0.02))
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftFrontServoDirection(CRServo.Direction.FORWARD)
            .rightFrontServoDirection(CRServo.Direction.FORWARD)
            .leftRearServoDirection(CRServo.Direction.FORWARD)
            .rightRearServoDirection(CRServo.Direction.FORWARD)
            .leftFrontPodAngleOffsetDeg(Math.toDegrees(2.7))
            .rightFrontPodAngleOffsetDeg(Math.toDegrees(3.5))
            .leftRearPodAngleOffsetDeg(Math.toDegrees(3.4))
            .rightRearPodAngleOffsetDeg(Math.toDegrees(1.17))
            .leftFrontPodXYOffsets(new double[] {126.075, 126})
            .rightFrontPodXYOffsets(new double[] {126.075, -126})
            .leftRearPodXYOffsets(new double[] {-126.075, 126})
            .rightRearPodXYOffsets(new double[] {-126.075, -126});


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
                .swerveDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
