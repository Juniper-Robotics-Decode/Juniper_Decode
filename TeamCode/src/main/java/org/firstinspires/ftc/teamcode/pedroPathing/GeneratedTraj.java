package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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


    public PathChain FarPath1;
    public PathChain FarPath2;
    public PathChain FarPath3;
    
    public PathChain ClosePath1;
    public PathChain ClosePath2;
    public PathChain ClosePath3;
    public PathChain ClosePath4;
    public PathChain ClosePath5;
    public PathChain ClosePath6;
    public PathChain ClosePath7;

    public Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private int transferSubState = 0;


    private Pose startPose = new Pose(120 ,127.87, Math.toRadians(319.6));

    // --- FAR PATH POSES ---
    // Start of Path 1 (Also usually your startPose)
    public Pose farStartPose = new Pose(89.000, 8.000, Math.toRadians(0));

    // Control point for the curve in Path 1
    public Pose farPath1Control = new Pose(80.000, 36.000);

    // End of Path 1 / Start of Path 2
    public Pose farPath1End = new Pose(105.000, 38.500, Math.toRadians(0));

    // End of Path 2 / Start of Path 3
    public Pose farPath2End = new Pose(130.000, 35.500, Math.toRadians(0));

    // End of Path 3
    public Pose farPath3End = new Pose(89.000, 12.000, Math.toRadians(0));

    private HWMap hwMap;
    private Logger logger;
    private RobotSettings robotSettings;
    private Pinpoint pinpoint;
    private IntakeFSM intakeFSM;
    private LauncherFSM launcherFSM;
    private TransferFSM transferFSM;



    public void ClosePaths(Follower follower) {
        ClosePath1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(120 ,127.87), new Pose(86.720, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(319.6), Math.toRadians(0))
                .build();

        ClosePath2 = follower
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

        ClosePath3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(105.000, 84.000), new Pose(130.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        ClosePath4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 84.000), new Pose(86.720, 90.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        ClosePath5 = follower
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

        ClosePath6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(105.000, 60.000), new Pose(130.000, 60.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        ClosePath7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 60.000), new Pose(86.720, 90.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

        public void farPaths(Follower follower) {

            if(robotSettings.alliance.equals(RobotSettings.Alliance.BLUE)) {
                mirrorFarPoses();
            }

            FarPath1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    farStartPose,
                                    farPath1Control,
                                    farPath1End
                            )
                    ).setLinearHeadingInterpolation(farStartPose.getHeading(), farPath1End.getHeading())
                    .build();

            FarPath2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    farPath1End,
                                    farPath2End
                            )
                    ).setLinearHeadingInterpolation(farPath1End.getHeading(), farPath2End.getHeading())
                    .build();

            FarPath3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    farPath2End,
                                    farPath3End
                            )
                    ).setLinearHeadingInterpolation(farPath2End.getHeading(), farPath3End.getHeading())
                    .build();
        }

    public void mirrorFarPoses() {
        // Assuming your Pedro Pathing branch supports a direct .mirror() or .flip() mutation
        farStartPose = farStartPose.mirror();
        farPath1Control = farPath1Control.mirror();
        farPath1End = farPath1End.mirror();
        farPath2End = farPath2End.mirror();
        farPath3End = farPath3End.mirror();
    }

    public void CloseSideUpdate() {
        pinpoint.update();
        launcherFSM.updateState(false,false,false,false,false,false,false,false,false,false,false,false);
        intakeFSM.updateState(false,false,false);
        switch (pathState) {
            case 0:
                transferFSM.updateState(false);
                follower.followPath(ClosePath1, true);
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
                    follower.followPath(ClosePath2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    transferFSM.updateState(false);
                    follower.followPath(ClosePath3, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    transferFSM.updateState(false);
                    follower.followPath(ClosePath4, true);
                    setPathState(6);
                }
                break;
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
                    follower.followPath(ClosePath5, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    transferFSM.updateState(false);
                    follower.followPath(ClosePath6, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    transferFSM.updateState(false);
                    follower.followPath(ClosePath7, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    transferFSM.updateState(false);
                    actionTimer.resetTimer();
                    pathState = 12;
                }
                break;
            case 12:
                if(actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    transferFSM.updateState(true);
                    pathState = 13;
                }
                break;
            case 13:
                    transferFSM.updateState(true);
                    if(transferFSM.TRANSFERED()) {
                        launcherFSM.setEndOfAuto(true);
                        setPathState(-1);
                    }
                break;
        }
    }


    public void FarSideUpdate() {
        pinpoint.update();
        launcherFSM.updateState(false,false,false,false,false,false,false,false,false,false,false,false);
        intakeFSM.updateState(false,false,false);
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    transferFSM.updateState(false);
                    actionTimer.resetTimer();
                    pathState = 1;
                }
                break;
            case 1:
                if(actionTimer.getElapsedTimeSeconds() >= 5) {
                    transferFSM.updateState(true);
                    pathState = 2;
                }
                break;
            case 2:
                switch (transferSubState) {
                    case 0: // Shot 1: Open
                        transferFSM.updateState(true);
                        if (transferFSM.TRANSFERED()) {
                            transferSubState = 1;
                        }
                        break;

                    case 1: // Shot 1: Close
                        transferFSM.updateState(false);
                        if (transferFSM.CLOSED()) {
                            actionTimer.resetTimer(); // Start delay timer for next shot
                            transferSubState = 2;
                        }
                        break;

                    case 2: // Shot 2: Wait & Open
                        if(actionTimer.getElapsedTimeSeconds() >= 1.0) { // 1-second delay
                            transferFSM.updateState(true);
                            if (transferFSM.TRANSFERED()) {
                                transferSubState = 3;
                            }
                        }
                        break;

                    case 3: // Shot 2: Close
                        transferFSM.updateState(false);
                        if (transferFSM.CLOSED()) {
                            actionTimer.resetTimer(); // Start delay timer for next shot
                            transferSubState = 4;
                        }
                        break;

                    case 4: // Shot 3: Wait & Open
                        if(actionTimer.getElapsedTimeSeconds() >= 1.0) { // 1-second delay
                            transferFSM.updateState(true);
                            if (transferFSM.TRANSFERED()) {
                                transferSubState = 5;
                            }
                        }
                        break;


                    case 5: // Shot 2: Close
                        transferFSM.updateState(false);
                        if (transferFSM.CLOSED()) {
                            actionTimer.resetTimer(); // Start delay timer for next shot
                            transferSubState = 6;
                        }
                        break;

                    case 6: // Shot 3: Wait & Open
                        if(actionTimer.getElapsedTimeSeconds() >= 1.0) { // 1-second delay
                            transferFSM.setTransferTime(100);
                            transferFSM.updateState(true);
                            if (transferFSM.TRANSFERED()) {
                                transferSubState = 7;
                            }
                        }
                        break;
                    case 7: // Shot 3: Close
                        transferFSM.updateState(false);
                        if (transferFSM.CLOSED()) {
                            // 3 Shots complete! Drive to next position
                            transferSubState = 0; // Reset for future use
                            follower.followPath(FarPath1, true);
                            setPathState(3);
                        }
                        break;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    transferFSM.updateState(false);
                    follower.followPath(FarPath2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    transferFSM.updateState(false);
                    follower.followPath(FarPath3, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    transferFSM.updateState(false);
                    actionTimer.resetTimer();
                    pathState = 6;
                }
                break;
            case 6:
                if(actionTimer.getElapsedTimeSeconds() >= 1.5) {
                    switch (transferSubState) {
                        case 0: // Shot 1: Open
                            transferFSM.setTransferTime(32);
                            transferFSM.updateState(true);
                            if (transferFSM.TRANSFERED()) {
                                transferSubState = 1;
                            }
                            break;

                        case 1: // Shot 1: Close
                            transferFSM.updateState(false);
                            if (transferFSM.CLOSED()) {
                                actionTimer.resetTimer(); // Start delay timer for next shot
                                transferSubState = 2;
                            }
                            break;

                        case 2: // Shot 2: Wait & Open
                            if(actionTimer.getElapsedTimeSeconds() >= 1.0) { // 1-second delay
                                transferFSM.updateState(true);
                                if (transferFSM.TRANSFERED()) {
                                    transferSubState = 3;
                                }
                            }
                            break;

                        case 3: // Shot 2: Close
                            transferFSM.updateState(false);
                            if (transferFSM.CLOSED()) {
                                actionTimer.resetTimer(); // Start delay timer for next shot
                                transferSubState = 4;
                            }
                            break;

                        case 4: // Shot 3: Wait & Open
                            if(actionTimer.getElapsedTimeSeconds() >= 1.0) { // 1-second delay
                                transferFSM.updateState(true);
                                if (transferFSM.TRANSFERED()) {
                                    transferSubState = 5;
                                }
                            }
                            break;


                        case 5: // Shot 2: Close
                            transferFSM.updateState(false);
                            if (transferFSM.CLOSED()) {
                                actionTimer.resetTimer(); // Start delay timer for next shot
                                transferSubState = 6;
                            }
                            break;

                        case 6: // Shot 3: Wait & Open
                            if(actionTimer.getElapsedTimeSeconds() >= 1.0) { // 1-second delay
                                transferFSM.setTransferTime(100);
                                transferFSM.updateState(true);
                                if (transferFSM.TRANSFERED()) {
                                    transferSubState = 7;
                                }
                            }
                            break;
                        case 7: // Shot 3: Close
                            transferFSM.updateState(false);
                            if (transferFSM.CLOSED()) {
                                // 3 Shots complete! Drive to next position
                                transferSubState = 0; // Reset for future use
                                follower.followPath(FarPath1, true);
                                setPathState(3);
                            }
                            break;
                    }
                }
                break;
            case 7:/*
                transferFSM.updateState(true);
                if (transferFSM.TRANSFERED()) {*/
                    launcherFSM.setEndOfAuto(true);
                    setPathState(-1);
               // }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robotSettings = RobotSettings.load();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        if(robotSettings.startPosState.equals(RobotSettings.StartPos.CLOSE_BLUE) || robotSettings.startPosState.equals(RobotSettings.StartPos.CLOSE_RED)) {
            telemetry.addData("close","");
            ClosePaths(follower);
        }
        else {
            telemetry.addData("far","");
            farPaths(follower);

        }

        startPose = new Pose(robotSettings.startPosState.getPose2D().getX(DistanceUnit.INCH),robotSettings.startPosState.getPose2D().getY(DistanceUnit.INCH), robotSettings.startPosState.getPose2D().getHeading(AngleUnit.RADIANS));
        follower.setStartingPose(startPose);

        opmodeTimer.resetTimer();
        setPathState(0);


        hwMap = new HWMap(hardwareMap);
        logger = new Logger(telemetry);
        robotSettings = RobotSettings.load();
        pinpoint = new Pinpoint(hwMap,robotSettings);
        launcherFSM = new LauncherFSM(hwMap,telemetry,pinpoint,robotSettings,logger, true);
        transferFSM = new TransferFSM(hwMap, telemetry,logger, true);
        intakeFSM = new IntakeFSM(hwMap, telemetry,logger);

        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            if(robotSettings.startPosState.equals(RobotSettings.StartPos.CLOSE_BLUE) || robotSettings.startPosState.equals(RobotSettings.StartPos.CLOSE_RED)) {
                CloseSideUpdate();
            }
            else {
                FarSideUpdate();
            }
            PoseStorage.currentPose = follower.getPose();

            launcherFSM.log();
            transferFSM.log();
            intakeFSM.log();
            telemetry.addData("path state", pathState);
            telemetry.addData("transfer states", transferSubState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("action timer", actionTimer.getElapsedTime());
            telemetry.update();
        }
    }
}

