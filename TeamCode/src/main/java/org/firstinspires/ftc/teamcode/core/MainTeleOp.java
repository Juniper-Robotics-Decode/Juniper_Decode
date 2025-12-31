/*
package org.firstinspires.ftc.teamcode.core;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Point;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.JoystickScaling;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.Swerve.Swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.shooter.ShooterFSM;
import org.firstinspires.ftc.teamcode.intake.IntakeFSM;
import org.firstinspires.ftc.teamcode.intaketransfer.TransferFSM;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class MainTeleOp extends LinearOpMode {
    private SwerveDrivetrain swerveDrivetrain;

    public double x, y, heading;
    public double BotHeading;
    public boolean locked;

    private Pose2D pos;

    private SlewRateLimiter XRate, YRate, HeadingRate;
    private JoystickScaling StrafingScaler, TurningScaler;


    private HWMap hwMap;
    private Pinpoint pinpoint;
    private RobotSettings robotSettings;
    private GamepadEx gamepad;
    private IntakeFSM intakeFSM;
    private TransferFSM transferFSM;
    private ShooterFSM shooterFSM;

    private Timing.Timer loopTimer;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        XRate = new SlewRateLimiter(xrate);
        YRate = new SlewRateLimiter(yrate);
        HeadingRate = new SlewRateLimiter(headingrate);
        StrafingScaler = new JoystickScaling();
        TurningScaler = new JoystickScaling();


        hwMap = new HWMap(hardwareMap);
        robotSettings = new RobotSettings();
        pinpoint = new Pinpoint(hwMap, robotSettings);

        swerveDrivetrain = new SwerveDrivetrain(hwMap);

        swerveDrivetrain.setOffsets(offsets);
        swerveDrivetrain.setInverses(inverses);
        swerveDrivetrain.setMotorScaling(scalars);

        intakeFSM = new IntakeFSM(hwMap, telemetry);
        transferFSM = new TransferFSM(hwMap, telemetry);
        shooterFSM = new ShooterFSM(hwMap,telemetry, pinpoint);

        loopTimer = new Timing.Timer(300000000, TimeUnit.MILLISECONDS);

        waitForStart();

        while (opModeIsActive()) {
            loopTimer.start();
            telemetry.addData("ALLIANCE", MainAuto.ALLIANCE);
            telemetry.addData("distance method", RobotSettings.distanceMethod);

            if (gamepad1.options) {
                pinpoint.resetPosAndIMU();
            }

            pinpoint.update();
            pos = pinpoint.getPos();
            BotHeading = -pos.getHeading(RADIANS);


            Pose drive = new Pose((StrafingScaler.ScaleVector(new Point(gamepad1.left_stick_x, -gamepad1.left_stick_y))), (-TurningScaler.Scale(gamepad1.right_stick_x, 0.01, 0.66, 4)));
            drive = new Pose(new Point(XRate.calculate(drive.x), YRate.calculate(drive.y)).rotate(BotHeading), HeadingRate.calculate(drive.heading));

            if (drive.x == 0 && drive.y == 0 && drive.heading == 0) {
                locked = true;
            }
            else {
                locked = false;
            }

            swerveDrivetrain.setLocked(locked);
            swerveDrivetrain.setPose(drive);
            swerveDrivetrain.updateModules();

           */
/* telemetry.addData("Bot Heading", BotHeading);
            telemetry.addData("Swerve Tele \n",swerveDrivetrain.getTele());*//*

            telemetry.addData("loop time", loopTimer.elapsedTime());
            intakeFSM.updateState(gamepad1.y, gamepad1.dpad_left);
            transferFSM.updateState(gamepad1.dpad_right, gamepad1.right_bumper);
            shooterFSM.updateState(gamepad1.b);
            shooterFSM.log();

            telemetry.update();
        }

    }

}

*/
