package org.firstinspires.ftc.teamcode.shooter.testClasses;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.headingrate;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.inverses;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.offsets;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.scalars;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.xrate;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.yrate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Point;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.JoystickScaling;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.Swerve.Swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Pinpoint;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.shooter.TurretFSM;
import org.firstinspires.ftc.teamcode.shooter.wrappers.LimelightCamera;

@TeleOp
public class RelocalizeTest extends LinearOpMode {
    HWMap hwMap;
    LimelightCamera limelightCamera;
    Pinpoint pinpoint;
    TurretFSM turretFSM;
    RobotSettings robotSettings;

    private SwerveDrivetrain swerveDrivetrain;


    public double BotHeading;
    public boolean locked;

    private Pose2D pos;

    private SlewRateLimiter XRate, YRate, HeadingRate;
    private JoystickScaling StrafingScaler, TurningScaler;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap);
        robotSettings = RobotSettings.load();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelightCamera = new LimelightCamera(hwMap.getLimelight(),telemetry, robotSettings);
        pinpoint = new Pinpoint(hwMap,robotSettings);
        turretFSM = new TurretFSM(hwMap,telemetry);


        XRate = new SlewRateLimiter(xrate);
        YRate = new SlewRateLimiter(yrate);
        HeadingRate = new SlewRateLimiter(headingrate);
        StrafingScaler = new JoystickScaling();
        TurningScaler = new JoystickScaling();

        swerveDrivetrain = new SwerveDrivetrain(hwMap);

        swerveDrivetrain.setOffsets(offsets);
        swerveDrivetrain.setInverses(inverses);
        swerveDrivetrain.setMotorScaling(scalars);

        telemetry.addLine("Alliance" + robotSettings.alliance);
        telemetry.addData("start Pos", robotSettings.startPosState.getPose2D());
        telemetry.update();


        waitForStart();
        while (opModeIsActive()) {
                limelightCamera.update();
                pinpoint.update();
                turretFSM.updateState();

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


            telemetry.addLine("-----------------");
            telemetry.addData("Limelight x", limelightCamera.getX());
            telemetry.addData("Limelight y", limelightCamera.getY());
            telemetry.addData("Limelight Ty", limelightCamera.getTy());

            double turretHeading = -turretFSM.getCurrentAngle();
            double camOffsetHeadingFromTurret = 48.064;
            double robotHeading = pinpoint.getHeading();
            double cameraAbsoluteHeading = robotHeading + turretHeading;

            double camX = limelightCamera.getxField();
            double camY = limelightCamera.getyField();
            double distanceCamToTurretCenter = 0.080724;

            double distanceTurretCenterToRobotCenter = 0.048;

            double X1 = distanceCamToTurretCenter*Math.cos(Math.toRadians(camOffsetHeadingFromTurret));
            double Y1 = distanceCamToTurretCenter*Math.sin(Math.toRadians(camOffsetHeadingFromTurret));

            double X2 = Y1*Math.sin(Math.toRadians(camOffsetHeadingFromTurret)) + X1*Math.cos(Math.toRadians(camOffsetHeadingFromTurret));
            double Y2 = Y1*Math.cos(Math.toRadians(camOffsetHeadingFromTurret)) + X1*Math.sin(Math.toRadians(camOffsetHeadingFromTurret));
            double xTurret =  camX + X2;
            double yTurret = camY + Y2;

            double xRobot = xTurret + (distanceTurretCenterToRobotCenter*(Math.cos(Math.toRadians(robotHeading))));
            double yRobot = yTurret + (distanceTurretCenterToRobotCenter*(Math.sin(Math.toRadians(robotHeading))));
            Pose2D newPos = new Pose2D(DistanceUnit.METER, xRobot,yRobot, AngleUnit.DEGREES, pinpoint.getHeading());

            if(gamepad1.a) {
                pinpoint.setPosition(newPos);
            }
            telemetry.addLine("--- RELOCALIZATION ---");
            telemetry.addData("Cam absolute Heading", cameraAbsoluteHeading);
            telemetry.addData("Robot Heading", robotHeading);
            telemetry.addData("Turret Heading", turretHeading);
            telemetry.addData("Turret X", xTurret);
            telemetry.addData("Turret Y", yTurret);
            telemetry.addData("Cam X", camX);
            telemetry.addData("Cam Y", camY);
            telemetry.addData("Robot X", xRobot);
            telemetry.addData("Robot Y", yRobot);
            telemetry.update();

        }
    }
}
