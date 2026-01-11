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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Point;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.JoystickScaling;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.Swerve.Drive.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.MainAuto;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.core.Pinpoint;

@TeleOp
public class PinpointDistanceTest extends LinearOpMode {

    private SwerveDrivetrain swerveDrivetrain;


    public double BotHeading;
    public boolean locked;

    private Pose2D pos;

    private SlewRateLimiter XRate, YRate, HeadingRate;
    private JoystickScaling StrafingScaler, TurningScaler;

    @Override
    public void runOpMode() {
        MainAuto.ALLIANCE = "RED";
        XRate = new SlewRateLimiter(xrate);
        YRate = new SlewRateLimiter(yrate);
        HeadingRate = new SlewRateLimiter(headingrate);
        StrafingScaler = new JoystickScaling();
        TurningScaler = new JoystickScaling();


        RobotSettings robotSettings = new RobotSettings();
        HWMap hwMap = new HWMap(hardwareMap);
        Pinpoint pinpoint = new Pinpoint(hwMap, robotSettings);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        swerveDrivetrain = new SwerveDrivetrain(hwMap);

        swerveDrivetrain.setOffsets(offsets);
        swerveDrivetrain.setInverses(inverses);
        swerveDrivetrain.setMotorScaling(scalars);

        telemetry.addLine("Alliance" + robotSettings.alliance);
        telemetry.addData("start Pos", robotSettings.startPosState.getPose2D());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
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

            swerveDrivetrain.setPose(drive, BotHeading, 12.4);
            swerveDrivetrain.updateModules();

            telemetry.addData("Bot Heading", BotHeading);
            telemetry.addData("Swerve Tele \n", swerveDrivetrain.getTele());

            telemetry.addData("RAW X", pinpoint.getX());
            telemetry.addData("RAW Y", pinpoint.getY());
            telemetry.addData("RAW Heading", pinpoint.getHeading());

            telemetry.addLine("-----------------");

            telemetry.addData("Distance to Goal", pinpoint.getGoalDistance());
            telemetry.addData("Heading Error", pinpoint.getHeadingErrorTrig());
            telemetry.addData("pinpoint ready", pinpoint.pinpointReady());

            String turnDir;
            if (pinpoint.getHeadingErrorTrig() > 0) {
                turnDir = "LEFT";
            }
            else {
                turnDir = "RIGHT";
            }
            if (Math.abs(pinpoint.getHeadingErrorTrig()) < 2.0) {
                turnDir = "AIMED";
            }

            telemetry.addData("Suggested Turn", turnDir);

            telemetry.update();
        }
    }
}
