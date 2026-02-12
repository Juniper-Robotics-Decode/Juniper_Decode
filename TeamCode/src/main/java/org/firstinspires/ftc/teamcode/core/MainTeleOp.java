package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.headingrate;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.inverses;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.offsets;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.scalars;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.xrate;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.yrate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Point;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.JoystickScaling;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.Swerve.Swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.shooter.LauncherFSM;
import org.firstinspires.ftc.teamcode.intake.IntakeFSM;
import org.firstinspires.ftc.teamcode.intaketransfer.TransferFSM;

import java.util.Locale;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "", group = "0")
public class MainTeleOp extends LinearOpMode {
    private SwerveDrivetrain swerveDrivetrain;

    public double x, y, heading;
    public boolean locked;

    private Pose2D pos;

    private SlewRateLimiter XRate, YRate, HeadingRate;
    private JoystickScaling StrafingScaler, TurningScaler;


    private HWMap hwMap;
    private Logger logger;

    private Pinpoint pinpoint;
    private RobotSettings robotSettings;
    private GamepadEx gamepad;
    private IntakeFSM intakeFSM;
    private TransferFSM transferFSM;
    private LauncherFSM launcherFSM;

    private Timing.Timer loopTimer;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        XRate = new SlewRateLimiter(xrate);
        YRate = new SlewRateLimiter(yrate);

        HeadingRate = new SlewRateLimiter(headingrate);
        StrafingScaler = new JoystickScaling();
        TurningScaler = new JoystickScaling();


        hwMap = new HWMap(hardwareMap);
        logger = new Logger(telemetry);
        robotSettings = new RobotSettings();
        pinpoint = new Pinpoint(hwMap, robotSettings);

        swerveDrivetrain = new SwerveDrivetrain(hwMap);

        swerveDrivetrain.setOffsets(offsets);
        swerveDrivetrain.setInverses(inverses);
        swerveDrivetrain.setMotorScaling(scalars);

        intakeFSM = new IntakeFSM(hwMap, logger);
        transferFSM = new TransferFSM(hwMap, logger);
        launcherFSM = new LauncherFSM(hwMap, logger, pinpoint);


        loopTimer = new Timing.Timer(300000000, TimeUnit.MILLISECONDS);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        waitForStart();

        while (opModeIsActive()) {
            loopTimer.start();
            if (Objects.equals(MainAuto.ALLIANCE, "RED")) {
                logger.log("<b><u><font color='red'>ALLIANCE</font></u></b>", MainAuto.ALLIANCE, Logger.LogLevels.PRODUCTION);
            } else if (Objects.equals(MainAuto.ALLIANCE, "BLUE")) {
                logger.log("<b><u><font color='blue'>ALLIANCE</font></u></b>", MainAuto.ALLIANCE, Logger.LogLevels.PRODUCTION);
            }
            logger.log("distance method", RobotSettings.distanceMethod, Logger.LogLevels.PRODUCTION);


            if (gamepad1.options) {
                pinpoint.resetPosAndIMU();
            }


            pinpoint.update();
            pos = pinpoint.getPos();
            double botHeading = -pos.getHeading(RADIANS);

            Pose drive = new Pose((StrafingScaler.ScaleVector(new Point(-gamepad1.left_stick_x, -gamepad1.left_stick_y))), (-TurningScaler.Scale(gamepad1.right_stick_x, 0.01, 0.66, 4)));
            drive = new Pose(new Point(XRate.calculate(drive.x), YRate.calculate(drive.y)).rotate(botHeading), HeadingRate.calculate(drive.heading));

            if (drive.x == 0 && drive.y == 0 && drive.heading == 0) {
                locked = true;
            } else {
                locked = false;
            }

            if (logger.PRODUCTION()) {
                intakeFSM.log();
                transferFSM.log();
                launcherFSM.log();
            } else if (logger.DEBUG() && gamepad2.left_bumper) {
                if (logger.LAUNCHER()) {
                    launcherFSM.log();
                } else if (logger.INTAKE_TRANSFER() && gamepad2.left_bumper) {
                    intakeFSM.log();
                    transferFSM.log();
                }
            }

            swerveDrivetrain.setLocked(locked);
            swerveDrivetrain.setPose(drive);
            swerveDrivetrain.updateModules();

            intakeFSM.updateState(gamepad1.y, gamepad1.dpad_left);
            transferFSM.updateState(gamepad1.dpad_right, gamepad1.right_bumper);
            launcherFSM.updateState(gamepad1.b);

            log();
            logger.print();
            logger.updateLoggingLevel(gamepad1.dpad_right);
            logger.updateMechanismLevel(gamepad1.left_bumper);
            logger.log("Bot Heading", botHeading, Logger.LogLevels.DEBUG);
        }
    }

    public void log() {
        logger.log("Swerve Tele \n", swerveDrivetrain.getTele(), Logger.LogLevels.DEBUG);
        logger.log("loop time", loopTimer.elapsedTime(), Logger.LogLevels.DEBUG);
    }
}

