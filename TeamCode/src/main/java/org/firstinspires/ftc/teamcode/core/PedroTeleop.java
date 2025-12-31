package org.firstinspires.ftc.teamcode.core;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.util.Timing;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Point;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.JoystickScaling;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.Swerve.Swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.intake.IntakeFSM;
import org.firstinspires.ftc.teamcode.intaketransfer.TransferFSM;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.shooter.ShooterFSM;

import java.util.concurrent.TimeUnit;

@TeleOp
public class PedroTeleop extends LinearOpMode {


    private Pose pos;

    private HWMap hwMap;
    private RobotSettings robotSettings;
    private GamepadEx gamepad;
    private Follower follower;
   /* private IntakeFSM intakeFSM;
    private TransferFSM transferFSM;
    private ShooterFSM shooterFSM;*/

    private Timing.Timer loopTimer;
    private TelemetryManager telemetryM;
    public static Pose startingPose;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose(): startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        hwMap = new HWMap(hardwareMap);
        robotSettings = new RobotSettings();

        /*
        pinpoint = new Pinpoint(hwMap, robotSettings);*/



        /*intakeFSM = new IntakeFSM(hwMap, telemetry);
        transferFSM = new TransferFSM(hwMap, telemetry);
        shooterFSM = new ShooterFSM(hwMap,telemetry, pinpoint);
*/
        loopTimer = new Timing.Timer(300000000, TimeUnit.MILLISECONDS);

        waitForStart();

        follower.startTeleOpDrive();
        telemetryM.update();
        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            follower.setTeleOpDrive(forward,strafe,turn,false);
            loopTimer.start();
            telemetry.addData("ALLIANCE", MainAuto.ALLIANCE);
            telemetry.addData("distance method", RobotSettings.distanceMethod);

            if (gamepad1.options) {
                follower.setPose(new Pose(0, 0, 0));
            }

            follower.update();
            pos = follower.getPose();


           /* telemetry.addData("Bot Heading", BotHeading);
            telemetry.addData("Swerve Tele \n",swerveDrivetrain.getTele());*/
            telemetry.addData("loop time", loopTimer.elapsedTime());
            /*intakeFSM.updateState(gamepad1.y, gamepad1.dpad_left);
            transferFSM.updateState(gamepad1.dpad_right, gamepad1.right_bumper);
            shooterFSM.updateState(gamepad1.b);
            shooterFSM.log();*/

            telemetryM.addData("Follower", follower.debug());
            telemetryM.update();
            telemetry.addData("Follow", follower.drivetrain.debugString());

            telemetry.update();
        }

    }

}
