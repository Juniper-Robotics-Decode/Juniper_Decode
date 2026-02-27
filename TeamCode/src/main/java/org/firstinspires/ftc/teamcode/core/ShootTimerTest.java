package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
//    Logger logger;
    GamepadEx gamepad;
    LimelightCamera limelight;
    Pinpoint pinpoint;
    LauncherFSM launcherFSM;
    TransferFSM transferFSM;
    IntakeFSM intakeFSM;
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    double timer = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        elapsedTime.reset();
        while(opModeIsActive()){
//            pinpoint.update();
//            gamepad.readButtons();
//            intakeFSM.updateState(gamepad1.dpad_up, gamepad1.dpad_left);
//            transferFSM.updateState(gamepad1.right_bumper);
//            launcherFSM.updateState(gamepad1.b,gamepad1.y,gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_right,gamepad2.y,gamepad2.a,gamepad2.b,gamepad2.x, gamepad2.left_bumper, gamepad2.right_bumper);
//            if(transferFSM.TRANSFERING() == true){
            if(gamepad1.dpad_up){
                    timer = 0;
                    elapsedTime.reset();
            }
                if(gamepad1.left_stick_button){
                    timer = elapsedTime.milliseconds();
                }
            Log.d("timer: ", String.valueOf(timer));
            Log.d("Etime: ", String.valueOf(elapsedTime));
//                logger.log("dist: ", limelight.getFlatDistance(), Logger.LogLevels.PRODUCTION);
        }
    }
}
