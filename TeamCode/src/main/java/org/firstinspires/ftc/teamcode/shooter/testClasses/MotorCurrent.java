package org.firstinspires.ftc.teamcode.shooter.testClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "", group = "5")
@Config
public class MotorCurrent extends LinearOpMode {

    DcMotorEx motor;
    public static double targetPower;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class,"FM");
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(targetPower);
            telemetry.addData("Current", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());


            double controlHubVoltage = 0;
            double expansionHubVoltage = 0;

            for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
                if (sensor.getDeviceName().contains("Control Hub")) {
                    controlHubVoltage = sensor.getVoltage();
                } else if (sensor.getDeviceName().contains("Expansion Hub")) {
                    expansionHubVoltage = sensor.getVoltage();
                }
            }

            telemetry.addData("Control Hub Voltage", controlHubVoltage);
            telemetry.addData("Expansion Hub Voltage", expansionHubVoltage);
            telemetry.addData("velocity ticks/sec", motor.getVelocity());
            telemetry.update();
        }

    }



}
