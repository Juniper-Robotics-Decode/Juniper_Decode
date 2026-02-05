package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Pinpoint;
import org.firstinspires.ftc.teamcode.core.PoseStorage;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.intake.IntakeFSM;
import org.firstinspires.ftc.teamcode.intaketransfer.TransferFSM;
import org.firstinspires.ftc.teamcode.shooter.ShooterFSM;

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
    private RobotSettings robotSettings;
    private Pinpoint pinpoint;
    private IntakeFSM intakeFSM;
    private ShooterFSM shooterFSM;
    private TransferFSM transferFSM;



    public void Paths(Follower follower) {
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
        shooterFSM.updateState(false);
        intakeFSM.updateState(false,false);
        switch (pathState) {
            case 0:
                transferFSM.updateState(false,false);
                follower.followPath(Path1, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    transferFSM.updateState(false,true);
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    transferFSM.updateState(false,false);
                    follower.followPath(Path3, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    transferFSM.updateState(false,true);
                    follower.followPath(Path4, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    transferFSM.updateState(false,false);
                    follower.followPath(Path5, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    transferFSM.updateState(false,false);
                    follower.followPath(Path6, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    transferFSM.updateState(false,false);
                    follower.followPath(Path7, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    transferFSM.updateState(false,true);
                    PoseStorage.currentPose = follower.getPose();
                    setPathState(-1);
                }
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
        follower = Constants.createFollower(hardwareMap);
        Paths(follower);
        follower.setStartingPose(startPose);

        opmodeTimer.resetTimer();
        setPathState(0);


        hwMap = new HWMap(hardwareMap);
        robotSettings = new RobotSettings();
        pinpoint = new Pinpoint(hwMap,robotSettings);
        intakeFSM = new IntakeFSM(hwMap, telemetry);
        transferFSM = new TransferFSM(hwMap, telemetry);
        shooterFSM = new ShooterFSM(hwMap,telemetry, pinpoint);

        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();


            shooterFSM.log();
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}

