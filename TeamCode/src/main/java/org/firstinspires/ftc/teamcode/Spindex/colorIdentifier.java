package org.firstinspires.ftc.teamcode.Spindex;

import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
public class colorIdentifier extends LinearOpMode {
    private RevColorSensorV3 colorSensor1;



    @Override
    public void runOpMode() {
        colorSensor1 = hardwareMap.get(RevColorSensorV3.class, "colorSensor1");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            int red1 = colorSensor1.red();
            int blue1 = colorSensor1.blue();
            int green1 = colorSensor1.green();
            double distance1 = colorSensor1.getDistance(DistanceUnit.MM);

            if (distance1 > 92) {
                telemetry.addData("Slot1(cs1): ", "Empty");
                telemetry.addData("rVAL", colorSensor1.red());
                telemetry.addData("gVAL", colorSensor1.green());
                telemetry.addData("bVAL", colorSensor1.blue());
            } else if (green1 > red1 && green1 > blue1) {
                telemetry.addData("Slot1(cs1): ", "Green");
                telemetry.addData("rVAL", colorSensor1.red());
                telemetry.addData("gVAL", colorSensor1.green());
                telemetry.addData("bVAL", colorSensor1.blue());
            } else if (blue1 > red1 && blue1 > green1) {
                telemetry.addData("Slot1(cs1): ", "Purple");
                telemetry.addData("rVAL", colorSensor1.red());
                telemetry.addData("gVAL", colorSensor1.green());
                telemetry.addData("bVAL", colorSensor1.blue());
            }
            telemetry.addData("distance", distance1);
            telemetry.update();
        }
    }
}
