
package org.firstinspires.ftc.teamcode.shooter.testClasses;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.Pinpoint;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.shooter.wrappers.LimelightCamera;

import java.util.Optional;

@TeleOp
public class PinpointLLRelocalizationTest extends LinearOpMode {
    HWMap hwMap;
    RobotSettings robotSettings;
    LimelightCamera limelightCamera;
    Pinpoint pinpoint;
    double X = 0;
    double Y = 0;
    double H = 0;
    double allianceYaw;
    Pose2D llPose;
    @Override
    public void runOpMode() throws InterruptedException {

        hwMap = new HWMap(hardwareMap);
        robotSettings = RobotSettings.load();
     //   telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        limelightCamera = new LimelightCamera(hwMap.getLimelight(),telemetry,robotSettings);
        pinpoint = new Pinpoint(hwMap,robotSettings,false);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                relocalize();
            }


            limelightCamera.update();
            X = limelightCamera.getxField();
            Y = limelightCamera.getyField();
            pinpoint.update();

            telemetry.addData("status","relocalizing");
            telemetry.addData("XL",X);
            telemetry.addData("YL",Y);


            telemetry.addData("status","relocalized");
            telemetry.addData("XP",pinpoint.getX());
            telemetry.addData("YP",pinpoint.getY());
            telemetry.update();
        }
    }
    public void relocalize(){
       /* if(limelightCamera.getTargetID() == 20){allianceYaw = -133.6;}
        else if(limelightCamera.getTargetID() == 24){allianceYaw = 133.6;}
        heading = limelightCamera.getTx() + allianceYaw;
*/
        llPose = new Pose2D(DistanceUnit.METER,X,Y,AngleUnit.DEGREES,pinpoint.getHeading());
        pinpoint.setPosition(llPose);

    }
}
