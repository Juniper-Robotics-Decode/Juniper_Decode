package org.firstinspires.ftc.teamcode.shooter.testClasses;

import android.os.Environment;

import com.opencsv.CSVWriter;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "", group = "2")
public class AngleTestCSV extends LinearOpMode {

    private Limelight3A limelight;
    private static IMU imu;
    private File file;
    private FileWriter fileWriter;
    private CSVWriter csvWriter;
    private final ArrayList<String[]> dataArray = new ArrayList<>();

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        initializeIMU();

        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1);

        // CSV file setup
        file = new File(String.format("%s/FIRST/AnglesData.csv",
                Environment.getExternalStorageDirectory().getAbsolutePath()));
        try {
            fileWriter = new FileWriter(file);
            csvWriter = new CSVWriter(fileWriter);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        // Header row
        dataArray.add(new String[]{"yaw (fid)", "yaw (tx)", "pitch (fid)","pitch (ty)"});

        waitForStart();

        for (int i = 0; i < 5000; i++) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                // Get position data from first detected fiducial
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {

                    LLResultTypes.FiducialResult fiducial = fiducials.get(0); // just takes first fiducial it sees - before had a for loop to find ID

                    double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    limelight.updateRobotOrientation(robotYaw);

                    double yaw = fiducial.getCameraPoseTargetSpace().getOrientation().getYaw(AngleUnit.DEGREES);
                    double pitch = fiducial.getCameraPoseTargetSpace().getOrientation().getPitch(AngleUnit.DEGREES);

                    double tx = result.getTx();
                    double ty = result.getTy();


                    telemetry.addData("tx", tx);
                    telemetry.addData("ty", ty);

                    telemetry.addData("i", i);
                    telemetry.addData("Fiducial ID", fiducial.getFiducialId());
                    telemetry.addData("Fid Yaw", yaw);
                    telemetry.addData("Fid Pitch", pitch);

                    dataArray.add(new String[]{
                            String.valueOf(yaw),
                            String.valueOf(tx),
                            String.valueOf(pitch),
                            String.valueOf(ty),
                    });

                } else {
                    telemetry.addData("Fiducials", "None detected");
                }

            } else {
                telemetry.addData("Limelight", "No valid result");
            }
            telemetry.update();
            sleep(10);
        }

        exportData();
    }

    private void exportData() {
        try {
            csvWriter.writeAll(dataArray);
            csvWriter.close();
        } catch (Exception ignored) {}
    }
    public static void initializeIMU() {
        RevHubOrientationOnRobot revHubOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        IMU.Parameters revParameters = new IMU.Parameters(revHubOrientation);
        imu.initialize(revParameters);
        imu.resetYaw();
    }
}
