package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.headingrate;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.inverses;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.offsets;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.scalars;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.xrate;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.yrate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Swerve.Drive.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Point;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.JoystickScaling;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.intaketransfer.TransferFSM;
import org.firstinspires.ftc.teamcode.shooter.LauncherFSM;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous
public class MainAuto extends LinearOpMode {


    public double x, y, heading;
    public boolean locked;


    private SlewRateLimiter XRate, YRate, HeadingRate;
    private JoystickScaling StrafingScaler, TurningScaler;


    HWMap hwMap;
    RobotSettings robotSettings;
    Pinpoint pinpoint;
    SwerveDrivetrain swerveDrivetrain;


    private Timing.Timer pathTimer1_CLOSE;
    private Timing.Timer pathTimer2_CLOSE;
    private Timing.Timer pathTimer1_FAR;
    private Timing.Timer transferTimer;


    public static double POWER1_CLOSE = 0.75;
    public static double POWER2_CLOSE = 0.75;
    public static long TIME1_CLOSE = 3000;
    public static long TIME2_CLOSE = 1000;


    public static double POWER1_FAR = 0.75;
    public static long TIME1_FAR = 3000;


    public static long TRANSFER_WAIT = 3000;
    int pathState = 0;

    LauncherFSM launcherFSM;
    TransferFSM transferFSM;
    double botHeading;

    Logger logger;

    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        XRate = new SlewRateLimiter(xrate);
        YRate = new SlewRateLimiter(yrate);


        HeadingRate = new SlewRateLimiter(headingrate);
        StrafingScaler = new JoystickScaling();
        TurningScaler = new JoystickScaling();


        hwMap = new HWMap(hardwareMap);
        robotSettings = RobotSettings.load();
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // pinpoint = new Pinpoint(hwMap,robotSettings, true);
        swerveDrivetrain = new SwerveDrivetrain(hwMap, logger);

        swerveDrivetrain.setOffsets(offsets);
        swerveDrivetrain.setInverses(inverses);
        swerveDrivetrain.setMotorScaling(scalars);

        pathTimer1_CLOSE = new Timing.Timer(TIME1_CLOSE, TimeUnit.MILLISECONDS);

        pathTimer2_CLOSE = new Timing.Timer(TIME2_CLOSE, TimeUnit.MILLISECONDS);

        pathTimer1_FAR = new Timing.Timer(TIME1_FAR, TimeUnit.MILLISECONDS);

        transferTimer = new Timing.Timer(TRANSFER_WAIT, TimeUnit.MILLISECONDS);

        logger = new Logger(telemetry);

        launcherFSM = new LauncherFSM(hwMap,telemetry,robotSettings,logger);

        transferFSM = new TransferFSM(hwMap,telemetry,logger);

        waitForStart();

        while (opModeIsActive()) {
            logger.updateLoggingLevel(gamepad1.touchpad);
            if(robotSettings.startPosState == RobotSettings.StartPos.CLOSE_RED || robotSettings.startPosState == RobotSettings.StartPos.CLOSE_BLUE) {
                closeAuto();
            }
            else {
                farSide();
            }
            launcherFSM.log();
            transferFSM.log();
            telemetry.update();
        }
    }

    public void closeAuto() {
        double voltage = hwMap.getVoltageSensor().getVoltage();

        if(robotSettings.alliance.getGoalPos().equals(RobotSettings.Alliance.BLUE.getGoalPos())) {
            botHeading = (-(imu.getRobotYawPitchRollAngles().getYaw(RADIANS) + robotSettings.startPosState.getPose2D().getHeading(RADIANS)) - Math.PI/2);
        }
        else {
            botHeading = (-(imu.getRobotYawPitchRollAngles().getYaw(RADIANS) + robotSettings.startPosState.getPose2D().getHeading(RADIANS)) + Math.PI/2);
        }


        launcherFSM.updateState(false,false,false,false,false,false,false,false,false,false,false,false, botHeading);
        switch (pathState) {
            case 0:
                transferFSM.updateState(false);
                if(!pathTimer1_CLOSE.isTimerOn()) {
                    pathTimer1_CLOSE.start();
                }

                Pose drive = new Pose((StrafingScaler.ScaleVector(new Point(0, POWER1_CLOSE))), (0));


                drive = new Pose(new Point(XRate.calculate(drive.x), YRate.calculate(drive.y)), HeadingRate.calculate(drive.heading));


                if (drive.x == 0 && drive.y == 0 && drive.heading == 0) {
                    locked = true;
                }
                else {
                    locked = false;
                }

                swerveDrivetrain.setPose(drive, botHeading, voltage);
                swerveDrivetrain.setLocked(locked);
                swerveDrivetrain.updateModules();
                if(pathTimer1_CLOSE.done()) {
                    pathState = 1;
                }
                break;
            case 1:
                Pose stopPose = new Pose(new Point(0, 0), 0);

                swerveDrivetrain.setPose(stopPose, botHeading, voltage);
                swerveDrivetrain.setLocked(locked);
                swerveDrivetrain.updateModules();

                transferFSM.updateState(true);
                if(transferFSM.TRANSFERED()) {
                    pathState = 2;
                }
                break;
            case 2:
                transferFSM.updateState(false);
                if(!pathTimer2_CLOSE.isTimerOn()) {
                    pathTimer2_CLOSE.start();
                }

                drive = new Pose((StrafingScaler.ScaleVector(new Point(POWER2_CLOSE, 0))), (0));


                drive = new Pose(new Point(XRate.calculate(drive.x), YRate.calculate(drive.y)).rotate(botHeading), HeadingRate.calculate(drive.heading));


                if (drive.x == 0 && drive.y == 0 && drive.heading == 0) {
                    locked = true;
                }
                else {
                    locked = false;
                }

                swerveDrivetrain.setPose(drive, botHeading, voltage);
                swerveDrivetrain.setLocked(locked);
                swerveDrivetrain.updateModules();

                if(pathTimer2_CLOSE.done()) {
                    pathTimer2_CLOSE.pause();
                    pathState = 3;
                }
                break;
            case 3:
                 stopPose = new Pose(new Point(0, 0), 0);

                swerveDrivetrain.setPose(stopPose, botHeading, voltage);
                swerveDrivetrain.setLocked(locked);
                swerveDrivetrain.updateModules();
                break;
        }

/*
            telemetry.addData("Bot Heading", botHeading);
            telemetry.addData("Swerve Tele \n",swerveDrivetrain.getTele());*/


        telemetry.addData("TIMER1", pathTimer1_CLOSE.elapsedTime());
        telemetry.addData("TIMER2", pathTimer2_CLOSE.elapsedTime());
        telemetry.addData("path state", pathState);



    }

    public void farSide() {
        telemetry.addData("FAAAAAAAAR", "");
        double voltage = hwMap.getVoltageSensor().getVoltage();


        if(robotSettings.alliance.getGoalPos().equals(RobotSettings.Alliance.BLUE.getGoalPos())) {
            botHeading = (-(imu.getRobotYawPitchRollAngles().getYaw(RADIANS) + robotSettings.startPosState.getPose2D().getHeading(RADIANS)) - Math.PI/2);
        }
        else {
            botHeading = (-(imu.getRobotYawPitchRollAngles().getYaw(RADIANS) + robotSettings.startPosState.getPose2D().getHeading(RADIANS)) + Math.PI/2);
        }

        launcherFSM.updateState(false, false, false, false, false, false, false, false, false, false, false, false, botHeading);

        switch (pathState) {
            case 0:
                transferFSM.updateState(false);
                if(!transferTimer.isTimerOn()) {
                    transferTimer.start();
                }
                if(transferTimer.done()) {
                    transferTimer.pause();
                    pathState = 1;
                }
                break;
            case 1:
                transferFSM.updateState(true);
                if(transferFSM.TRANSFERED()) {
                    pathState = 2;
                }
                break;
            case 2:
                transferFSM.updateState(false);
                if (!pathTimer1_FAR.isTimerOn()) {
                    pathTimer1_FAR.start();
                }

                Pose drive = new Pose((StrafingScaler.ScaleVector(new Point(POWER1_FAR, 0))), (0));


                drive = new Pose(new Point(XRate.calculate(drive.x), YRate.calculate(drive.y)), HeadingRate.calculate(drive.heading));


                if (drive.x == 0 && drive.y == 0 && drive.heading == 0) {
                    locked = true;
                } else {
                    locked = false;
                }

                swerveDrivetrain.setPose(drive, botHeading, voltage);
                swerveDrivetrain.setLocked(locked);
                swerveDrivetrain.updateModules();
                if (pathTimer1_FAR.done()) {
                    pathTimer1_FAR.pause();
                    pathState = 3;
                }
                break;
            case 3:
                transferFSM.updateState(false);
                Pose stopPose = new Pose(new Point(0, 0), 0);

                swerveDrivetrain.setPose(stopPose, botHeading, voltage);
                swerveDrivetrain.setLocked(locked);
                swerveDrivetrain.updateModules();
                pathState = 4;
                break;

            case 4:
                transferFSM.updateState(false);
                launcherFSM.setEndOfAuto(true);
                Pose stopPose2 = new Pose(new Point(0, 0), 0);

                swerveDrivetrain.setPose(stopPose2, botHeading, voltage);
                swerveDrivetrain.setLocked(locked);
                swerveDrivetrain.updateModules();

                break;
        }

        telemetry.addData("TIMER", pathTimer1_FAR.elapsedTime());
        telemetry.addData("path state", pathState);
        telemetry.addData("Transfer Wait", transferTimer.elapsedTime());

    }

    public void farRedSide() {
        telemetry.addData("REDDD FAAAAAAAAR", "");
        double voltage = hwMap.getVoltageSensor().getVoltage();


        if(robotSettings.alliance.getGoalPos().equals(RobotSettings.Alliance.BLUE.getGoalPos())) {
            botHeading = (-(imu.getRobotYawPitchRollAngles().getYaw(RADIANS) + robotSettings.startPosState.getPose2D().getHeading(RADIANS)) - Math.PI/2);
        }
        else {
            botHeading = (-(imu.getRobotYawPitchRollAngles().getYaw(RADIANS) + robotSettings.startPosState.getPose2D().getHeading(RADIANS)) + Math.PI/2);
        }

        launcherFSM.updateState(false, false, false, false, false, false, false, false, false, false, false, false, botHeading);

        switch (pathState) {
            case 0:
                transferFSM.updateState(false);
                if(!transferTimer.isTimerOn()) {
                    transferTimer.start();
                }
                if(transferTimer.done()) {
                    transferTimer.pause();
                    pathState = 1;
                }
                break;
            case 1:
                transferFSM.updateState(true);
                if(transferFSM.TRANSFERED()) {
                    pathState = 2;
                }
                break;
            case 2:
                transferFSM.updateState(false);
                if (!pathTimer1_FAR.isTimerOn()) {
                    pathTimer1_FAR.start();
                }

                Pose drive = new Pose((StrafingScaler.ScaleVector(new Point(POWER1_FAR, 0))), (0));


                drive = new Pose(new Point(XRate.calculate(drive.x), YRate.calculate(drive.y)), HeadingRate.calculate(drive.heading));


                if (drive.x == 0 && drive.y == 0 && drive.heading == 0) {
                    locked = true;
                } else {
                    locked = false;
                }

                swerveDrivetrain.setPose(drive, botHeading, voltage);
                swerveDrivetrain.setLocked(locked);
                swerveDrivetrain.updateModules();
                if (pathTimer1_FAR.done()) {
                    pathTimer1_FAR.pause();
                    pathState = 3;
                }
                break;
            case 3:
                transferFSM.updateState(false);
                Pose stopPose = new Pose(new Point(0, 0), 0);

                swerveDrivetrain.setPose(stopPose, botHeading, voltage);
                swerveDrivetrain.setLocked(locked);
                swerveDrivetrain.updateModules();
                pathState = 4;
                break;

            case 4:
                transferFSM.updateState(false);
                launcherFSM.setEndOfAuto(true);
                Pose stopPose2 = new Pose(new Point(0, 0), 0);

                swerveDrivetrain.setPose(stopPose2, botHeading, voltage);
                swerveDrivetrain.setLocked(locked);
                swerveDrivetrain.updateModules();

                break;
        }

        telemetry.addData("TIMER", pathTimer1_FAR.elapsedTime());
        telemetry.addData("path state", pathState);
        telemetry.addData("Transfer Wait", transferTimer.elapsedTime());

    }
}
