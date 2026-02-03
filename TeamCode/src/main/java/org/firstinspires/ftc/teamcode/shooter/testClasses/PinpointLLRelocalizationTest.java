//package org.firstinspires.ftc.teamcode.shooter.testClasses;
//
//import android.util.Log;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.core.Logger;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.core.Pinpoint;
//import org.firstinspires.ftc.teamcode.shooter.wrappers.LimelightCamera;
//
//@TeleOp
//public class PinpointLLRelocalizationTest extends LinearOpMode {
//    LimelightCamera limelightCamera;
//    Pinpoint pinpoint;
//    double X = 0;
//    double Y = 0;
//    double heading = 0;
//    double allianceYaw;
//    Pose2D llPose;
//    Logger logger;
//    Telemetry telemetry;
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        logger = new Logger(telemetry);
//        while (opModeIsActive()) {
//            if (
//                    gamepad1.x) {
//                logger.log("status","relocalizing", Logger.LogLevels.DEBUG);
//                logger.log("X1",X, Logger.LogLevels.DEBUG);
//                logger.log("Y1",Y, Logger.LogLevels.DEBUG);
//                logger.log("H1",heading, Logger.LogLevels.DEBUG);
//                relocalize();
//                logger.log("status","relocalized", Logger.LogLevels.DEBUG);
//                logger.log("X2",X, Logger.LogLevels.DEBUG);
//                logger.log("Y2",Y, Logger.LogLevels.DEBUG);
//                logger.log("H2",heading, Logger.LogLevels.DEBUG);
//            }
//            logger.log("status","none", Logger.LogLevels.DEBUG);
//            logger.log("X",X, Logger.LogLevels.DEBUG);
//            logger.log("Y",Y, Logger.LogLevels.DEBUG);
//            logger.log("H",heading, Logger.LogLevels.DEBUG);
//        }
//    }
//    public void relocalize(){
//        limelightCamera.update();
//        X = limelightCamera.getxField();
//        Y = limelightCamera.getyField();
//        if(limelightCamera.getTargetID() == 20){allianceYaw = -133.6;}
//        else if(limelightCamera.getTargetID() == 24){allianceYaw = 133.6;}
//        heading = limelightCamera.getTx() + allianceYaw;
//        llPose = new Pose2D(DistanceUnit.METER,X,Y,AngleUnit.DEGREES,heading);
//        pinpoint.setPosition(llPose);
//        pinpoint.update();
//    }
//}
