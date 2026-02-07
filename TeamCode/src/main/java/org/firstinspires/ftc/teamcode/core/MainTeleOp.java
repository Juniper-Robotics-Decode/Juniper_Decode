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
import com.arcrobotics.ftclib.gamepad.GamepadEx;
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
    private GamepadEx gamepad;
    private IntakeFSM intakeFSM;
    private TransferFSM transferFSM;
    private LauncherFSM launcherFSM;

    public static double P, I, D, F;


    private Timing.Timer loopTimer;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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


        loopTimer = new Timing.Timer(300000000, TimeUnit.MILLISECONDS);
        double botHeading;
        waitForStart();

        while (opModeIsActive()) {
            loopTimer.start();
            logger.log("ALLIANCE", robotSettings.alliance, Logger.LogLevels.PRODUCTION);
            logger.log("distance method", robotSettings.distanceMethod, Logger.LogLevels.PRODUCTION);

            swerveDrivetrain.setHeadingControllerPIDF(P, I, D, F);

            double voltage = hwMap.getVoltageSensor().getVoltage();

            logger.updateLoggingLevel(gamepad1.touchpad);

            if (gamepad1.options) {
                pinpoint.resetIMU();
            }
            if(gamepad2.right_bumper) {
                pinpoint.resetPos();
            }

            if(gamepad1.back && robotSettings.distanceMethod == RobotSettings.DistanceMethod.PINPOINT_ONLY) {
                 robotSettings.distanceMethod = RobotSettings.DistanceMethod.LIMELIGHT_ONLY;
            }

            if(gamepad1.back && robotSettings.distanceMethod == RobotSettings.DistanceMethod.LIMELIGHT_ONLY) {
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

         //   logger.log("is blue", robotSettings.alliance.getGoalPos().equals(RobotSettings.Alliance.BLUE.getGoalPos()), Logger.LogLevels.PRODUCTION);
            logger.log("Bot Heading", botHeading, Logger.LogLevels.DEBUG);
            logger.log("loop time", loopTimer.elapsedTime(), Logger.LogLevels.DEBUG);
            logger.log("battery voltage", voltage, Logger.LogLevels.DEBUG);
            intakeFSM.updateState(gamepad1.y, gamepad1.dpad_left);
            transferFSM.updateState(gamepad1.right_bumper);
            launcherFSM.updateState(gamepad1.b,gamepad1.dpad_up,gamepad1.left_bumper,gamepad2.dpad_up,gamepad2.dpad_down,gamepad2.dpad_left,gamepad2.dpad_right,gamepad2.y,gamepad2.a,gamepad2.b,gamepad2.x, gamepad2.left_bumper, gamepad2.right_bumper);

            intakeFSM.log();
            transferFSM.log();
            launcherFSM.log();
            swerveDrivetrain.log();
            telemetry.addData("Pose", drive);

            telemetry.update();
        }

    }

}

