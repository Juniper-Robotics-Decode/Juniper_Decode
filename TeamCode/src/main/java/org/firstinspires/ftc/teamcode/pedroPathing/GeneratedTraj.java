package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.core.Pinpoint;
import org.firstinspires.ftc.teamcode.core.PoseStorage;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.intake.IntakeFSM;
import org.firstinspires.ftc.teamcode.intaketransfer.TransferFSM;
import org.firstinspires.ftc.teamcode.shooter.LauncherFSM;

@Autonomous
public class GeneratedTraj extends LinearOpMode {

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;

    public Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;


    private final Pose startPose = new Pose(86.720, 137.685, Math.toRadians(0));

    private HWMap hwMap;
    private Logger logger;
    private RobotSettings robotSettings;
    private Pinpoint pinpoint;
    private IntakeFSM intakeFSM;
    private LauncherFSM launcherFSM;
    private TransferFSM transferFSM;



    public void Paths(Follower follower) {
       /* Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(86.720, 137.685), new Pose(86.720, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.720, 90.000),
                                new Pose(86.720, 84.000),
                                new Pose(105.000, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();*/



        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(86.720, 137.685), new Pose(86.720, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.720, 90.000),
                                new Pose(86.720, 84.000),
                                new Pose(105.000, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(105.000, 84.000), new Pose(130.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 84.000), new Pose(86.720, 90.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.720, 90.000),
                                new Pose(100.000, 60.000),
                                new Pose(105.000, 60.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(105.000, 60.000), new Pose(130.000, 60.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 60.000), new Pose(86.720, 90.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        pinpoint.update();
        launcherFSM.updateState(false,false,false,false,false,false,false,false,false,false,false,false,false);
        intakeFSM.updateState(false,false);
        switch (pathState) {
            case 0:
                transferFSM.updateState(false);
                follower.followPath(Path1, true);
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy()) {
                    transferFSM.updateState(false);
                    actionTimer.resetTimer();
                    pathState = 2;
                }
                break;
            case 2:
                if(actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    transferFSM.updateState(true);
                    pathState = 3;
                }
                break;
            case 3:
                transferFSM.updateState(true);
                if (transferFSM.TRANSFERED()) {
                    follower.followPath(Path2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    transferFSM.updateState(false);
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    transferFSM.updateState(false);
                    follower.followPath(Path4, true);
                    setPathState(6);
                }
            case 6:
                if(!follower.isBusy()) {
                    transferFSM.updateState(false);
                    actionTimer.resetTimer();
                    pathState = 7;
                }
                break;
            case 7:
                if(actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    transferFSM.updateState(true);
                    pathState = 8;
                }
                break;
            case 8:
                transferFSM.updateState(true);
                if (transferFSM.TRANSFERED()) {
                    follower.followPath(Path5, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    transferFSM.updateState(false);
                    follower.followPath(Path6, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    transferFSM.updateState(false);
                    follower.followPath(Path7, true);
                    setPathState(11);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    transferFSM.updateState(false);
                    actionTimer.resetTimer();
                    pathState = 13;
                }
                break;
            case 13:
                if(actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    transferFSM.updateState(true);
                    pathState = 14;
                }
                break;
            case 14:
                    transferFSM.updateState(true);
                    launcherFSM.setEndOfAuto(true);
                    PoseStorage.currentPose = follower.getPose();
                    setPathState(-1);
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        Paths(follower);
        follower.setStartingPose(startPose);

        opmodeTimer.resetTimer();
        setPathState(0);


        hwMap = new HWMap(hardwareMap);
        logger = new Logger(telemetry);
        robotSettings = RobotSettings.load();
        pinpoint = new Pinpoint(hwMap,robotSettings, false);
        launcherFSM = new LauncherFSM(hwMap,telemetry,pinpoint,robotSettings,logger, true);
        transferFSM = new TransferFSM(hwMap, telemetry,logger);
        intakeFSM = new IntakeFSM(hwMap, telemetry,transferFSM,logger);

        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();


            launcherFSM.log();
            transferFSM.log();
            intakeFSM.log();
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("action timer", actionTimer.getElapsedTime());
            telemetry.update();
        }
    }
}

