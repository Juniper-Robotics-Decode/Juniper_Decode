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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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

    private Pose2D pos;

    private SlewRateLimiter XRate, YRate, HeadingRate;
    private JoystickScaling StrafingScaler, TurningScaler;


    HWMap hwMap;
    RobotSettings robotSettings;
    Pinpoint pinpoint;
    SwerveDrivetrain swerveDrivetrain;


    private Timing.Timer pathTimer1_CLOSE;
    private Timing.Timer pathTimer2_CLOSE;
    private Timing.Timer pathTimer1_FAR;


    public static double POWER1_CLOSE = 0.75;
    public static double POWER2_CLOSE = 0.75;
    public static long TIME1_CLOSE = 3000;
    public static long TIME2_CLOSE = 1000;


    public static double POWER1_FAR = 0.75;
    public static long TIME1_FAR = 2000;

    int pathState = 0;

    LauncherFSM launcherFSM;
    TransferFSM transferFSM;

    Logger logger;

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
        pinpoint = new Pinpoint(hwMap,robotSettings, true);
        swerveDrivetrain = new SwerveDrivetrain(hwMap);

        swerveDrivetrain.setOffsets(offsets);
        swerveDrivetrain.setInverses(inverses);
        swerveDrivetrain.setMotorScaling(scalars);

        pathTimer1_CLOSE = new Timing.Timer(TIME1_CLOSE, TimeUnit.MILLISECONDS);

        pathTimer2_CLOSE = new Timing.Timer(TIME2_CLOSE, TimeUnit.MILLISECONDS);

        pathTimer1_FAR = new Timing.Timer(TIME1_FAR, TimeUnit.MILLISECONDS);

        logger = new Logger(telemetry);

        launcherFSM = new LauncherFSM(hwMap,telemetry,pinpoint,robotSettings,logger);

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

        pinpoint.update();
        pos = pinpoint.getPos();
        double botHeading = -pos.getHeading(RADIANS);

        launcherFSM.updateState(false,false,false,false,false,false,false,false,false,false,false,false,false);
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
                Pose stopPose = new Pose(new Point(0, 0), 0);

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
        double voltage = hwMap.getVoltageSensor().getVoltage();

        pinpoint.update();
        pos = pinpoint.getPos();
        double botHeading = -pos.getHeading(RADIANS);

        launcherFSM.updateState(false, false, false, false, false, false, false, false, false, false, false, false, false);

        switch (pathState) {
            case 0:
                transferFSM.updateState(true);
                if(transferFSM.TRANSFERED()) {
                    pathState = 1;
                }
                break;
            case 1:
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
                    pathState = 2;
                }
                break;
            case 2:
                Pose stopPose = new Pose(new Point(0, 0), 0);

                swerveDrivetrain.setPose(stopPose, botHeading, voltage);
                swerveDrivetrain.setLocked(locked);
                swerveDrivetrain.updateModules();
                break;
        }
        telemetry.addData("TIMER", pathTimer1_FAR.elapsedTime());
        telemetry.addData("path state", pathState);

        }
    }


