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
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Point;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.JoystickScaling;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.Swerve.Drive.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.shooter.LauncherFSM;
import org.firstinspires.ftc.teamcode.intake.IntakeFSM;
import org.firstinspires.ftc.teamcode.intaketransfer.TransferFSM;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class MainTeleOp extends LinearOpMode {
    private SwerveDrivetrain swerveDrivetrain;

    public double x, y, heading;
    public boolean locked;

    private Pose2D pos;

    private SlewRateLimiter XRate, YRate, HeadingRate;
    private JoystickScaling StrafingScaler, TurningScaler;

    private Logger logger;
    private HWMap hwMap;
    private Pinpoint pinpoint;
    private RobotSettings robotSettings;
    private GamepadEx gamepadE2;
    private GamepadEx gamepadE1;
    private IntakeFSM intakeFSM;
    private TransferFSM transferFSM;
    private LauncherFSM launcherFSM;

    public static double P, I, D, F;


    private Timing.Timer loopTimer;


    @Override
    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        gamepadE2 = new GamepadEx(gamepad2);
        gamepadE1 = new GamepadEx(gamepad1);

        XRate = new SlewRateLimiter(xrate);
        YRate = new SlewRateLimiter(yrate);

        HeadingRate = new SlewRateLimiter(headingrate);
        StrafingScaler = new JoystickScaling();
        TurningScaler = new JoystickScaling();

        logger = new Logger(telemetry);
        hwMap = new HWMap(hardwareMap);
        robotSettings = RobotSettings.load();
        pinpoint = new Pinpoint(hwMap, robotSettings, false);

        swerveDrivetrain = new SwerveDrivetrain(hwMap, logger);

        swerveDrivetrain.setOffsets(offsets);
        swerveDrivetrain.setInverses(inverses);
        swerveDrivetrain.setMotorScaling(scalars);

        transferFSM = new TransferFSM(hwMap, telemetry,logger);
        intakeFSM = new IntakeFSM(hwMap, telemetry,transferFSM,logger);
        launcherFSM = new LauncherFSM(hwMap,telemetry, pinpoint, robotSettings, logger);

        P = 0.008; I = 0; D = 0;

        loopTimer = new Timing.Timer(300000000, TimeUnit.MILLISECONDS); double botHeading;
        logger.log("<b><u><i><font color='orange'>CHECK THE CHECKLIST</font></i></b></u>","", Logger.LogLevels.PRODUCTION);
        if(robotSettings.alliance == RobotSettings.Alliance.RED) {
            logger.log("<b><u><font color='red'>ALLIANCE</font></u></b>", robotSettings.alliance, Logger.LogLevels.PRODUCTION);
        }else{
            logger.log("<b><u><font color='blue'>ALLIANCE</font></u></b>", robotSettings.alliance, Logger.LogLevels.PRODUCTION);
        }
        logger.log("<b><u><i><font color='orange'>distance method</font></b></u></i>", robotSettings.distanceMethod, Logger.LogLevels.PRODUCTION);
        telemetry.update();
        waitForStart();
       while (opModeIsActive()) {
            loopTimer.start();
            gamepadE2.readButtons();
            swerveDrivetrain.setHeadingControllerPIDF(P, I, D, F);

            double voltage = hwMap.getVoltageSensor().getVoltage();

            logger.updateLoggingLevel(gamepadE1.getButton(GamepadKeys.Button.LEFT_BUMPER));

            if (gamepadE1.getButton(GamepadKeys.Button.START)) {
                pinpoint.resetIMU();
            }

            if(gamepadE1.getButton(GamepadKeys.Button.BACK) && robotSettings.distanceMethod == RobotSettings.DistanceMethod.PINPOINT_ONLY) {
                 robotSettings.distanceMethod = RobotSettings.DistanceMethod.LIMELIGHT_ONLY;
            }

            if(gamepadE1.getButton(GamepadKeys.Button.BACK) && robotSettings.distanceMethod == RobotSettings.DistanceMethod.LIMELIGHT_ONLY) {
                robotSettings.distanceMethod = RobotSettings.DistanceMethod.LIMELIGHT_ONLY;
            }

            pinpoint.update();
            pos = pinpoint.getPos();
            if(robotSettings.alliance.getGoalPos().equals(RobotSettings.Alliance.BLUE.getGoalPos())) {
                botHeading = (-pos.getHeading(RADIANS)) - Math.PI/2;
            }
            else {
                botHeading = (-pos.getHeading(RADIANS)) + Math.PI/2;
            }

            Pose drive = new Pose((StrafingScaler.ScaleVector(new Point(-gamepadE1.getLeftX(), -gamepadE1.getLeftY()))), (TurningScaler.Scale(gamepadE1.getRightX(), 0.01, 0.66, 4)));
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
            intakeFSM.updateState(gamepadE1.getButton(GamepadKeys.Button.DPAD_UP), gamepadE1.getButton(GamepadKeys.Button.DPAD_LEFT));
            transferFSM.updateState(gamepadE1.getButton(GamepadKeys.Button.RIGHT_BUMPER));
            launcherFSM.updateState(gamepadE1.getButton(GamepadKeys.Button.B),gamepadE1.getButton(GamepadKeys.Button.Y),gamepadE2.wasJustPressed(GamepadKeys.Button.DPAD_UP), gamepadE2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN),gamepadE2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT),gamepadE2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT),gamepadE2.wasJustPressed(GamepadKeys.Button.Y),gamepadE2.wasJustPressed(GamepadKeys.Button.A),gamepadE2.wasJustPressed(GamepadKeys.Button.B),gamepadE2.wasJustPressed(GamepadKeys.Button.X), gamepadE2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER), gamepadE2.getButton(GamepadKeys.Button.RIGHT_BUMPER));

         //   logger.log("is blue", robotSettings.alliance.getGoalPos().equals(RobotSettings.Alliance.BLUE.getGoalPos()), Logger.LogLevels.PRODUCTION);
            logUpdate(botHeading, voltage, drive);

            telemetry.update();
        }

    }
    private void logUpdate(double botHeading, double voltage, Pose drive){
        launcherFSM.positionFSM.logPP();
        launcherFSM.positionFSM.logLL();
        launcherFSM.flywheelFSM.log();
        launcherFSM.pitchFSM.log();
        launcherFSM.turretFSM.log();
        launcherFSM.positionFSM.log();
        swerveDrivetrain.log();
        transferFSM.log();
        intakeFSM.log();
        telemetry.addData("Pose", drive);
        logger.log("battery voltage", voltage, Logger.LogLevels.DEBUG);
        logger.log("loop time", loopTimer.elapsedTime(), Logger.LogLevels.DEBUG);
        logger.log("Bot Heading", botHeading, Logger.LogLevels.DEBUG);
    }

}

