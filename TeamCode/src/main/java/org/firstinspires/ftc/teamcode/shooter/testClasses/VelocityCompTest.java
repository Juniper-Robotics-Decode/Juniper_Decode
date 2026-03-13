/*
package org.firstinspires.ftc.teamcode.shooter.testClasses;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.headingrate;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.inverses;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.offsets;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.scalars;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.xrate;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.yrate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Swerve.Drive.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Point;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.JoystickScaling;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.core.Pinpoint;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.intake.IntakeFSM;
import org.firstinspires.ftc.teamcode.intaketransfer.TransferFSM;
import org.firstinspires.ftc.teamcode.shooter.LauncherFSM;
import org.firstinspires.ftc.teamcode.shooter.wrappers.LimelightCamera;
import org.slf4j.LoggerFactory;

@TeleOp
public class VelocityCompTest extends LinearOpMode{
    SwerveDrivetrain swerveDrivetrain;
    HWMap hwMap;
    Logger logger;
    RobotSettings robotSettings;
    GamepadEx gamepad;
    LauncherFSM launcherFSM;
    IntakeFSM intakeFSM;
    TransferFSM transferFSM;
    Pinpoint pinpoint;
    private Pose2D pos;
boolean locked;
    private SlewRateLimiter XRate, YRate, HeadingRate;
    private JoystickScaling StrafingScaler, TurningScaler;
    LimelightCamera limelight;
    public static double P, I, D, F;
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    double vXNew, vYNew, aX, aY, t, tLast, tTotal = 0;
    double d;
    double botHeading;
    double tShoot = 0.5;//TODO: make this the time it takes to launch an artifact in seconds
    double timeConstant = 10;//t=27.5+0.247x+2.55x^2-1.49x^3
    double goalXFinalOffset, goalYFinalOffset, goalXShootOffset, goalYShootOffset, turretOffset = 0;
    @Override
    public void runOpMode() throws InterruptedException {
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            logger = new Logger(telemetry);
            hwMap = new HWMap(hardwareMap);
            robotSettings = RobotSettings.load();
            pinpoint = new Pinpoint(hwMap, robotSettings,false);
            limelight = new LimelightCamera(hwMap.getLimelight(), telemetry, robotSettings);
            gamepad = new GamepadEx(gamepad1);
        launcherFSM = new LauncherFSM(hwMap,telemetry, pinpoint,robotSettings,logger);
        transferFSM = new TransferFSM(hwMap,telemetry,logger);
        intakeFSM = new IntakeFSM(hwMap,telemetry,transferFSM,logger);
        XRate = new SlewRateLimiter(xrate);
        YRate = new SlewRateLimiter(yrate);

        HeadingRate = new SlewRateLimiter(headingrate);
        StrafingScaler = new JoystickScaling();
        TurningScaler = new JoystickScaling();

        swerveDrivetrain = new SwerveDrivetrain(hwMap, logger);
            swerveDrivetrain.setOffsets(offsets);
            swerveDrivetrain.setInverses(inverses);
            swerveDrivetrain.setMotorScaling(scalars);
        P = 0.008; I = 0; D = 0;

        waitForStart();
        while(opModeIsActive()) {
            logger.updateLoggingLevel(gamepad1.left_bumper);
            swerveDrivetrain.setHeadingControllerPIDF(P, I, D, F);pinpoint.update();
            pos = pinpoint.getPos();
            if(robotSettings.alliance.getGoalPos().equals(RobotSettings.Alliance.BLUE.getGoalPos())) {
                botHeading = (-pos.getHeading(RADIANS)) - Math.PI/2;
            }
            else {
                botHeading = (-pos.getHeading(RADIANS)) + Math.PI/2;
            }

            Pose drive = new Pose((StrafingScaler.ScaleVector(new Point(-gamepad1.left_stick_x, gamepad1.left_stick_y))), (TurningScaler.Scale(gamepad1.right_stick_x, 0.01, 0.66, 4)));
            drive = new Pose(new Point(XRate.calculate(drive.x), YRate.calculate(drive.y)).rotate(botHeading), HeadingRate.calculate(drive.heading));

            if (drive.x == 0 && drive.y == 0 && drive.heading == 0) {
                locked = true;
            }
            else {
                locked = false;
            }

            swerveDrivetrain.setPose(drive, botHeading, 12.4);
            swerveDrivetrain.setLocked(locked);
            swerveDrivetrain.updateModules();
//            double vXLast = vXNew;
//            double vYLast = vYNew;
//            tLast = t;
            pinpoint.update();
            gamepad.readButtons();
            intakeFSM.updateState(gamepad1.dpad_up, gamepad1.dpad_left);
            transferFSM.updateState(gamepad1.right_bumper);
            launcherFSM.updateState(gamepad1.b, gamepad1.dpad_up, gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_right, gamepad2.y, gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.left_bumper, gamepad2.right_bumper);
            vXNew = pinpoint.getXV();
            vYNew = pinpoint.getYV();
//            aX = (vXNew)/tLast;
//            aY = (vYNew)/tLast;
//            tTotal = launcherFSM.flywheelFSM.getTargetVelocityRPM()/timeConstant;
//            goalXShootOffset = (-vXNew * tShoot) + (-aX * Math.pow(tShoot,2)/2);
            d = pinpoint.getGoalDistance();
            tTotal = (27.5+(0.247*d)+(2.55*(Math.pow(d,2)))-(1.49*(Math.pow(d,3))))/1000;
//            goalYShootOffset = (-vYNew * tShoot) + (-aY * Math.pow(tShoot,2)/2);
//            double goalDist = pinpoint.getGoalDistanceAdjusted(goalXShootOffset,goalYShootOffset);
//            double vO = launcherFSM.positionFSM.getFlywheelTargetVelocity(goalDist);
//            double theta = launcherFSM.pitchFSM.getTargetAngle() * 12;
//            double vFlat = Math.cos(theta) * vO;
//            double tTravel = goalDist/vFlat;
            goalXFinalOffset = (-vXNew * tTotal);
            goalYFinalOffset = (-vYNew * tTotal);

            turretOffset = -pinpoint.getHV() * tTotal;
            launcherFSM.turretFSM.setVelocityOffset(turretOffset);
            pinpoint.setVelocityOffsets(goalXFinalOffset,goalYFinalOffset);
//            t = elapsedTime.seconds() - tLast;
            log();
            telemetry.update();
        }
    }
    private void log(){
        logger.log("X Shoot Offset", goalXShootOffset, Logger.LogLevels.PRODUCTION);
        logger.log("Y Shoot Offset", goalYShootOffset, Logger.LogLevels.PRODUCTION);
        logger.log("X Final Offset", goalXFinalOffset, Logger.LogLevels.PRODUCTION);
        logger.log("Y Final Offset", goalYFinalOffset, Logger.LogLevels.PRODUCTION);
        logger.log("Turret Offset", turretOffset, Logger.LogLevels.PRODUCTION);
        logger.log("Time", t, Logger.LogLevels.PRODUCTION);
        logger.log("Last Time", tLast, Logger.LogLevels.PRODUCTION);
        logger.log("X Acceleration", aX, Logger.LogLevels.PRODUCTION);
        logger.log("Y Acceleration", aY, Logger.LogLevels.PRODUCTION);
        logger.log("X Velocity", vXNew, Logger.LogLevels.PRODUCTION);
        logger.log("Y Velocity", vYNew, Logger.LogLevels.PRODUCTION);
//        logger.log("time ms", elapsedTime.time(), Logger.LogLevels.PRODUCTION);
        logger.log("time total", tTotal, Logger.LogLevels.PRODUCTION);
        launcherFSM.positionFSM.logPP();
        launcherFSM.positionFSM.logLL();
        launcherFSM.flywheelFSM.log();
        launcherFSM.pitchFSM.log();
        launcherFSM.turretFSM.log();
        launcherFSM.positionFSM.log();
        swerveDrivetrain.log();
        transferFSM.log();
        intakeFSM.log();
    }
}
*/
