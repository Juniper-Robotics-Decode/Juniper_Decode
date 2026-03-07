package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.intaketransfer.TransferFSM;
import org.firstinspires.ftc.teamcode.intake.IntakeFSM;
import org.firstinspires.ftc.teamcode.shooter.LauncherFSM;
import org.firstinspires.ftc.teamcode.shooter.wrappers.LimelightCamera;

@TeleOp
public class ShootTimerTest extends LinearOpMode{
    HWMap hwMap;
    Logger logger;
    GamepadEx gamepad;
    RobotSettings robotSettings;
//    LimelightCamera limelight;
    Pinpoint pinpoint;
    LauncherFSM launcherFSM;

    TransferFSM transferFSM;
    IntakeFSM intakeFSM;
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    double timer = 0;
    int n = 0;
    double avg = 0;
    double total = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        logger = new Logger(telemetry);
        hwMap = new HWMap(hardwareMap);
        robotSettings = RobotSettings.load();
        pinpoint = new Pinpoint(hwMap, robotSettings,false);
//        limelight = new LimelightCamera(hwMap.getLimelight(), telemetry, robotSettings);
        gamepad = new GamepadEx(gamepad1);
        launcherFSM = new LauncherFSM(hwMap,telemetry, pinpoint,robotSettings,logger);
        transferFSM = new TransferFSM(hwMap,telemetry,logger);
        intakeFSM = new IntakeFSM(hwMap,telemetry,transferFSM,logger);
        waitForStart();
        waitForStart();
        elapsedTime.reset();
        while(opModeIsActive()){
            double d = pinpoint.getGoalDistance();//limelight.getFlatDistance();
            double tTotal = 27.5+(0.247*d)+(2.55*(Math.pow(d,2)))-(1.49*(Math.pow(d,3)));
            logger.updateLoggingLevel(gamepad1.left_bumper);
            pinpoint.update();
            gamepad.readButtons();
            intakeFSM.updateState(gamepad1.dpad_up, gamepad1.dpad_left);
            transferFSM.updateState(gamepad1.right_bumper);
            launcherFSM.updateState(gamepad1.b,gamepad1.y,gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_right,gamepad2.y,gamepad2.a,gamepad2.b,gamepad2.x, gamepad2.left_bumper, gamepad2.right_bumper);
//            while(transferFSM.TRANSFERING() == true){
//            if(gamepad1.dpad_up){
//                    timer = 0;
//                    elapsedTime.reset();
//
//            }
            if(gamepad1.right_bumper || gamepad1.right_stick_button){
                timer = 0;
                elapsedTime.reset();
                telemetry.addData("RIGHT","");
            }
            if(gamepad1.left_stick_button){
                timer = elapsedTime.milliseconds();
                telemetry.addData("LEFT","");
                total += timer;
                avg = total/(n+=1);
                elapsedTime.reset();
                logger.log("t", timer, Logger.LogLevels.PRODUCTION);
                logger.log("x",pinpoint.getGoalDistance(), Logger.LogLevels.PRODUCTION);
            }
            logger.log("Timer: ", timer, Logger.LogLevels.PRODUCTION);
            logger.log("Etime: ", elapsedTime, Logger.LogLevels.PRODUCTION);;
            logger.log("Avg: ", avg, Logger.LogLevels.PRODUCTION);
            logger.log("dist: ", pinpoint.getGoalDistance(), Logger.LogLevels.PRODUCTION);
            logger.log("tTotal",tTotal, Logger.LogLevels.PRODUCTION);
            launcherFSM.positionFSM.logPP();
            launcherFSM.positionFSM.logLL();
            launcherFSM.flywheelFSM.log();
            launcherFSM.pitchFSM.log();
            launcherFSM.turretFSM.log();
            launcherFSM.positionFSM.log();
            transferFSM.log();
            intakeFSM.log();
            telemetry.update();
        }
    }
}
