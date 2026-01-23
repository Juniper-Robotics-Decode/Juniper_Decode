package org.firstinspires.ftc.teamcode.Spindex;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.opencsv.CSVWriter;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

@TeleOp
public class colorSensorCVS extends LinearOpMode {
    private RevColorSensorV3 colorSensor1;
//    private RevColorSensorV3 colorSensor2;
//    private RevColorSensorV3 colorSensor3;


    private ElapsedTime loopTimer;

    private DcMotor spindexMotor;

    private File file1;
    //    private File file2;
//    private File file3;
    private FileWriter fileWriter1;
    private FileWriter fileWriter2;
    private FileWriter fileWriter3;
    private CSVWriter csvWriter1;
    private CSVWriter csvWriter2;
    private CSVWriter csvWriter3;
    private boolean addData = true;

    @Override
    public void runOpMode() {
        spindexMotor = hardwareMap.get(DcMotor.class, "spindexMotor");
        colorSensor1 = hardwareMap.get(RevColorSensorV3.class, "colorSensor1");
        //      colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor2");
        //      colorSensor3 = hardwareMap.get(RevColorSensorV3.class, "colorSensor3");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        file1 = new File(String.format("%s/FIRST/ColorSensor2Green.csv", Environment.getExternalStorageDirectory().getAbsolutePath()));
        //      file2 = new File(String.format("%s/FIRST/BlackNewSpindexPartDisturbance.csv", Environment.getExternalStorageDirectory().getAbsolutePath()));
        //      file3 = new File(String.format("%s/FIRST/SlipRingNoiseTest3.csv", Environment.getExternalStorageDirectory().getAbsolutePath()));

        waitForStart();
        //   spindexMotor.setPower(0.16);
        if (addData) {
            colorSensorDataCollection(colorSensor1, file1);
            //   colorSensorDataCollection(colorSensor2, file2);
            //   colorSensorDataCollection(colorSensor3, file3);
            addData = false;
        }
        else{
            telemetry.addData("-", "DATA COLLECTING HAS FINISHED");
            telemetry.update();
        }


    }
    public void colorSensorDataCollection(RevColorSensorV3 cs, File file ) {
        ArrayList<String[]> dataArray = new ArrayList<String[]>();
        dataArray.add(new String[]{"Red", "Green", "Blue", "Alpha", "Distance"});
        FileWriter fileWriter;
        try {
            fileWriter = new FileWriter(file);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        CSVWriter csvWriter = new CSVWriter(fileWriter);
        for (int i = 0; i < 1000; i++) {
            // spindexMotor.setPower(0.16);
            int red = cs.red();
            int blue = cs.blue();
            int green = cs.green();
            int alpha = cs.alpha();
            double distance = cs.getDistance(DistanceUnit.MM);

            telemetry.addData("Red:", red);
            telemetry.addData("Green:", green);
            telemetry.addData("Blue:", blue);
            telemetry.addData("Alpha:", alpha);
            telemetry.addData("Distance:", distance);
            telemetry.addData("i:", i);
            telemetry.update();

            String Red = red + "";
            String Green = green + "";
            String Blue = blue + "";
            String Alpha = alpha + "";
            String Distance = distance + "";

            dataArray.add(new String[]{Red, Green, Blue, Alpha, Distance});
        }

        try {
            csvWriter.writeAll(dataArray);
            csvWriter.close();
        } catch (Exception e) {
            telemetry.addData("-", e.getMessage());
            telemetry.update();
        }

    }

}