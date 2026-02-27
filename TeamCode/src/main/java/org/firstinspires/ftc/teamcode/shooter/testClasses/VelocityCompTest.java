package org.firstinspires.ftc.teamcode.shooter.testClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.core.Pinpoint;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.shooter.LauncherFSM;
import org.slf4j.LoggerFactory;

@TeleOp
public class VelocityCompTest extends LinearOpMode{
    HWMap hwMap;
    Logger logger;
    RobotSettings robotSettings;
    GamepadEx gamepad;
    LauncherFSM launcherFSM;
    Pinpoint pinpoint;
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    double vXNew, vYNew, aX, aY, t, tLast, tTotal = 0;
    double tShoot = 0.5;//TODO: make this the time it takes to launch an artifact in seconds
    double timeConstant = 10;
    double goalXFinalOffset, goalYFinalOffset, goalXShootOffset, goalYShootOffset, turretOffset = 0;
    @Override
    public void runOpMode() throws InterruptedException {
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            logger = new Logger(telemetry);
            hwMap = new HWMap(hardwareMap);
            robotSettings = RobotSettings.load();
            pinpoint = new Pinpoint(hwMap, robotSettings,false);
            gamepad = new GamepadEx(gamepad1);
            launcherFSM = new LauncherFSM(hwMap,telemetry, pinpoint,robotSettings,logger);
            waitForStart();
        while(opModeIsActive()) {
            double vXLast = vXNew;
            double vYLast = vYNew;
            tLast = t;
            pinpoint.update();
            gamepad.readButtons();
            launcherFSM.updateState(gamepad1.b, gamepad1.dpad_up, gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_right, gamepad2.y, gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.left_bumper, gamepad2.right_bumper);
            vXNew = pinpoint.getXV();
            vYNew = pinpoint.getYV();
            aX = (vXNew)/tLast;
            aY = (vYNew)/tLast;
            tTotal = launcherFSM.flywheelFSM.getTargetVelocityRPM()/timeConstant;
//            goalXShootOffset = (-vXNew * tShoot) + (-aX * Math.pow(tShoot,2)/2);
//            goalYShootOffset = (-vYNew * tShoot) + (-aY * Math.pow(tShoot,2)/2);
//            double goalDist = pinpoint.getGoalDistanceAdjusted(goalXShootOffset,goalYShootOffset);
//            double vO = launcherFSM.positionFSM.getFlywheelTargetVelocity(goalDist);
//            double theta = launcherFSM.pitchFSM.getTargetAngle() * 12;
//            double vFlat = Math.cos(theta) * vO;
//            double tTravel = goalDist/vFlat;
            goalXFinalOffset = -vXNew * tTotal;
            goalYFinalOffset = -vYNew * tTotal;
            turretOffset = -pinpoint.getHV() * tShoot;
            launcherFSM.turretFSM.setVelocityOffset(turretOffset);
            pinpoint.setVelocityOffsets(goalXFinalOffset,goalYFinalOffset);
            t = elapsedTime.seconds() - tLast;
            log();
        }
    }
    private void log(){
        logger.log("X Shoot Offset", goalXShootOffset, Logger.LogLevels.DEBUG);
        logger.log("Y Shoot Offset", goalYShootOffset, Logger.LogLevels.DEBUG);
        logger.log("X Final Offset", goalXFinalOffset, Logger.LogLevels.DEBUG);
        logger.log("Y Final Offset", goalYFinalOffset, Logger.LogLevels.DEBUG);
        logger.log("Turret Offset", turretOffset, Logger.LogLevels.DEBUG);
        logger.log("Time", t, Logger.LogLevels.DEBUG);
        logger.log("Last Time", tLast, Logger.LogLevels.DEBUG);
        logger.log("X Acceleration", aX, Logger.LogLevels.DEBUG);
        logger.log("Y Acceleration", aY, Logger.LogLevels.DEBUG);
        logger.log("X Velocity", vXNew, Logger.LogLevels.DEBUG);
        logger.log("Y Velocity", vYNew, Logger.LogLevels.DEBUG);
        logger.log("time ms", elapsedTime.time(), Logger.LogLevels.DEBUG);
    }
}
