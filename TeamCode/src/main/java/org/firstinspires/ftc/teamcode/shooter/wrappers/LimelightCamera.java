package org.firstinspires.ftc.teamcode.shooter.wrappers;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.MainAuto;
import org.firstinspires.ftc.teamcode.core.RobotSettings;

import java.util.List;

public class LimelightCamera {

    private Limelight3A limelight;

    private boolean hasValidTarget = false;
    private double x_m = 0;
    private double y_m = 0;
    private double z_m = 0;
    private double xField = 0;
    private double yField = 0;
    private int targetID = 0;
    private double flatDistance_m = 0;
    private double tx_degrees = 0;
    private double ty_degrees = 0;
    private Telemetry telemetry;
    private RobotSettings robotSettings;

    public LimelightCamera(Limelight3A limelight3A, Telemetry telemetry, RobotSettings robotSettings) {
        limelight = limelight3A;
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1);
        this.telemetry = telemetry;
        this.robotSettings = robotSettings;
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        telemetry.addData("is there a limelight result", !(result == null));
        telemetry.addData("is there a valid limelight result", result.isValid());
        if (result == null) { // || !result.isValid()
            hasValidTarget = false;
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        telemetry.addData("is there a fiducial result", !(fiducials == null));
        telemetry.addData("is there a non-empty fiducial", !fiducials.isEmpty());
        if (fiducials == null || fiducials.isEmpty()) {
            hasValidTarget = false;
            return;
        }

        LLResultTypes.FiducialResult fiducial = null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if(robotSettings.alliance == RobotSettings.Alliance.RED) {
                if (f.getFiducialId() == 24) {
                    fiducial = f;
                    break;
                }
            }

            else if (robotSettings.alliance == RobotSettings.Alliance.BLUE) {
                if (f.getFiducialId() == 20) {
                    fiducial = f;
                    break;
                }
            }


        }

        telemetry.addData("was the target ID found", !(fiducial==null));
        if (fiducial == null) {
            hasValidTarget = false;
            return;
        }

        hasValidTarget = true;
        targetID = fiducial.getFiducialId();


        x_m = fiducial.getCameraPoseTargetSpace().getPosition().x;
        y_m = fiducial.getCameraPoseTargetSpace().getPosition().y;
        z_m = fiducial.getCameraPoseTargetSpace().getPosition().z;
        tx_degrees = result.getTx();
        ty_degrees = result.getTy();

        xField = fiducial.getRobotPoseFieldSpace().getPosition().x;
        yField = fiducial.getRobotPoseFieldSpace().getPosition().y;

        flatDistance_m = Math.sqrt(x_m * x_m + z_m * z_m);
    }


    public boolean hasTarget() {
        return hasValidTarget;
    }

    public double getX() {
        return x_m;
    }

    public double getY() {
        return y_m;
    }

    public double getZ() {
        return z_m;
    }

    public double getTx() {
        return tx_degrees;
    }

    public double getTy() {return ty_degrees;}

    public double getFlatDistance() {
        return flatDistance_m;
    }

    public double getxField() {
        return xField;
    }

    public double getyField() {
        return yField;
    }

    public int getTargetID() {
        return targetID;
    }

    public double getGoalDistance() {
        return Math.sqrt(Math.pow((robotSettings.alliance.getGoalPos().getX(DistanceUnit.METER) - xField), 2) + Math.pow((robotSettings.alliance.getGoalPos().getY(DistanceUnit.METER) - yField), 2));
    }

    public double getHeadingErrorTrig(double heading) {
        double targetAngle;
        targetAngle = Math.toDegrees(Math.atan2((robotSettings.alliance.getGoalPos().getY(DistanceUnit.METER) - yField), (robotSettings.alliance.getGoalPos().getX(DistanceUnit.METER) - xField)));

        double error = targetAngle - heading;

        if(error <= -180) {
            error += 360;
        }
        else if (error >= 180) {
            error -= 360;
        }
       /* error = -error;
        error = 360 - error;*/
        return error;
    }





}