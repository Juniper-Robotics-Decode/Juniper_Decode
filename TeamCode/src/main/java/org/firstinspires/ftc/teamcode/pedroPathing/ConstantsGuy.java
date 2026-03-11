/*
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ConstantsGuy {
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-117.118)
            .lateralZeroPowerAcceleration(-117.118)
            .mass(7.801789)

            //rrv:2.788 lfv: 2.611 lrv: 3.1 rfv: 0.681
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.005 , 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.003, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0 , 0.003, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.63, 0, 0.035, 0))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0035, 0.0, 0.0003, 0.6, 0))
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFSwitch(5)
            .headingPIDFSwitch(0.1)
            .centripetalScaling(0.0005);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5.2165354) // -132.5 mm
            .strafePodX(0.554133858) // 14.074 mm
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static CustomSwerveConstants swerveConstants = new CustomSwerveConstants();

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .setDrivetrain(new CustomSwerveDrivetrain(hardwareMap, swerveConstants))
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
*/
