/*
package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class GeneratedTraj extends LinearOpMode {

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;


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
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    telemetry.addData("followed", 0);
                }
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
        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();


            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}

*/
