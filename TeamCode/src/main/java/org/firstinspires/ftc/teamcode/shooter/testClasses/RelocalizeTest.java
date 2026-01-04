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
    double CAMERA_DISTANCE_FROM_CENTER = 0.07207114;

    private SwerveDrivetrain swerveDrivetrain;


    public double BotHeading;
    public boolean locked;

    private Pose2D pos;

    private SlewRateLimiter XRate, YRate, HeadingRate;
    private JoystickScaling StrafingScaler, TurningScaler;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap);
        robotSettings = new RobotSettings();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelightCamera = new LimelightCamera(hwMap.getLimelight(),telemetry);
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

            swerveDrivetrain.setLocked(locked);
            swerveDrivetrain.setPose(drive);
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


            //TODO: add 20 degree pitch and add 90 degree roll
                double cameraAbsoluteHeading = pinpoint.getHeading() - turretFSM.getCurrentAngle();
                double vectorAbsoluteHeading = cameraAbsoluteHeading - limelightCamera.getTy();
                double vectorMagnitude = limelightCamera.getFlatDistance();

                double xCam = RobotSettings.alliance.getGoalPos().getX(DistanceUnit.METER) - (vectorMagnitude*(Math.cos(Math.toRadians(vectorAbsoluteHeading))));
                double yCam = RobotSettings.alliance.getGoalPos().getY(DistanceUnit.METER) - (vectorMagnitude*(Math.sin(Math.toRadians(vectorAbsoluteHeading))));

                double xRobot = xCam + (CAMERA_DISTANCE_FROM_CENTER*  (Math.cos(Math.toRadians(cameraAbsoluteHeading))));
                double yRobot = yCam + (CAMERA_DISTANCE_FROM_CENTER*(Math.sin(Math.toRadians(cameraAbsoluteHeading))));



                if(gamepad1.a) {
                    Pose2D newPos = new Pose2D(DistanceUnit.METER, xRobot, yRobot, AngleUnit.DEGREES, pinpoint.getHeading());
                    pinpoint.setPosition(newPos);
                }

                telemetry.addLine("--- RELOCALIZATION ---");
                telemetry.addData("Cam absolute Heading", cameraAbsoluteHeading);
                telemetry.addData("Pinpoint heading", pinpoint.getHeading());
                telemetry.addData("turret current angle", turretFSM.getCurrentAngle());
                telemetry.addData("Vector Heading", vectorAbsoluteHeading);
                telemetry.addData("Vector Dist", vectorMagnitude);
                telemetry.addData("Cam X", xCam);
                telemetry.addData("Cam Y", yCam);
                telemetry.addData("Robot X", xRobot);
                telemetry.addData("Robot Y", yRobot);
                telemetry.update();

        }
    }
}
