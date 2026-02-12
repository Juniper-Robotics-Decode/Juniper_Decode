package org.firstinspires.ftc.teamcode.shooter.testClasses;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "", group = "3")
public class LimeLightTest extends LinearOpMode {

    private Limelight3A limelight;
    private ElapsedTime loopTimer;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // 100 updates/sec
        limelight.start();
        limelight.pipelineSwitch(0);

        loopTimer = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {
            loopTimer.reset();
            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();

            updateTelemetry(status, result);

            telemetry.addData("Loop Time (ms)", loopTimer.milliseconds());
            telemetry.update();
        }
    }

    //tx,ty,ta stuff
    public double[] getTargetData(LLResult result) {
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();
            return new double[]{tx, ty, ta};
        }
        return new double[]{0,0,0};
    }

    // MegaTag poses returned - kind of used to create objects of both version 1 & 2

    public Pose3D getMegaTag1Pose(LLResult result) {
        if (result != null && result.isValid()) {
            return result.getBotpose();
        }
        return null;
    }

    public Pose3D getMegaTag2Pose(LLResult result) {
        if (result != null && result.isValid()) {
            return result.getBotpose_MT2();
        }
        return null;
    }

    // Next two methods find distances & orientation w/ mega tag 1 or 2

    public double getXYDistanceToGoal(Pose3D pose) {
        if (pose != null) {
            // Robot's position
            double robotX = pose.getPosition().x;
            double robotY = pose.getPosition().y;

            // Goal's position (Red Goal center)
            double goalX = 0; // TODO: Change
            double goalY = 0;

            double distanceX = goalX - robotX;
            double distanceY = goalY - robotY;
            return Math.sqrt(distanceX * distanceX + distanceY * distanceY);
        }
        return 0;
    }


    public double[] getOrientation3D(Pose3D pose) {
        if (pose != null) {
            double yaw = pose.getOrientation().getYaw(AngleUnit.DEGREES);
            double pitch = pose.getOrientation().getPitch(AngleUnit.DEGREES);
            double roll = pose.getOrientation().getRoll(AngleUnit.DEGREES);

            return new double[]{yaw, pitch, roll};
        }
        return new double[]{0, 0, 0};
    }


    /* --------------------  Own Algorithim -------------------- */

    private static final double CAMERA_HEIGHT_M = 0.2794;  // 11 in
    private static final double TARGET_HEIGHT_M = 0; // TODO: Change
    private static final double CAMERA_PITCH_DEG = 24.0; // may not be best data

    public double getXDistance_ty(LLResult result) {
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToTarget = Math.toRadians(CAMERA_PITCH_DEG + ty);
            return (TARGET_HEIGHT_M - CAMERA_HEIGHT_M) / Math.tan(angleToTarget);
        }
        return 0;
    }

    public double getXYDistance_ty(LLResult result) {
        double x = getXDistance_ty(result);
        double y = TARGET_HEIGHT_M - CAMERA_HEIGHT_M;
        return Math.sqrt(x*x + y*y);
    }



    /* --------------------  Telemetry -------------------- */

    public void updateTelemetry(LLStatus status, LLResult result) {
        telemetry.addData("Name", status.getName());
        telemetry.addData("Pipeline", status.getPipelineIndex());
        telemetry.addData("FPS", status.getFps());

        if (result != null && result.isValid()) {
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("ta", result.getTa());

            // MegaTag1
            Pose3D mt1 = getMegaTag1Pose(result);
            if (mt1 != null) {
                telemetry.addData("MT1 XY Distance", getXYDistanceToGoal(mt1));
            }

            // MegaTag2
            Pose3D mt2 = getMegaTag2Pose(result);
            if (mt2 != null) {
                telemetry.addData("MT2 XY Distance", getXYDistanceToGoal(mt2));
                double[] ypr = getOrientation3D(mt2);
                telemetry.addData("Yaw", ypr[0]);
                telemetry.addData("Pitch", ypr[1]);
                telemetry.addData("Roll", ypr[2]);
            }

            // ty-based
            telemetry.addData("Distance (ty)", getXYDistance_ty(result));


        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }
}
