package org.firstinspires.ftc.teamcode.shooter.testClasses;

import android.os.Environment;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.opencsv.CSVWriter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;

@Config
@TeleOp(name="FlywheelPIDTestChanges")
public class FlyWheelPIDTestChanges extends LinearOpMode {

    DcMotorEx wheel;
//
    public static double P = 0.0001;
    public static double I = 0.0012;
    public static double D = 0.0002;
    public static double F = 0.00019;

    public static double errorBoostThreshold = 200;
    public static double errorBoostPower = 0.30;
    public static double targetRPM = 2650;

    double integralSum = 0;
    double lastError = 0;
    double lastDerivative = 0;
    double lastBoostTime = -1;
    double TICKS_PER_REV = 28;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();
    ArrayList<String[]> dataLog = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {

        wheel = hardwareMap.get(DcMotorEx.class, "FM");
        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dataLog.add(new String[]{"Time", "Target", "Actual", "Error", "Power", "Boost"});

        waitForStart();
        timer.reset();
        runtime.reset();

        while (opModeIsActive()) {
            double dt = timer.seconds();
            if (dt <= 0) dt = 0.001;
            timer.reset();

            double actualTPS = Math.abs(wheel.getVelocity());
            double actualRPM = (actualTPS / TICKS_PER_REV) * 60;
            double error = targetRPM - actualRPM;

            if (gamepad1.x) {
                integralSum = 0;
                lastError = 0;
            }

            double errorBoost = 0;
            if (error > errorBoostThreshold || (runtime.seconds() - lastBoostTime < 0.15)) {
                errorBoost = errorBoostPower;
                if (error > errorBoostThreshold) {
                    lastBoostTime = runtime.seconds();
                }
            }
            if (Math.abs(error) < 250 && errorBoost == 0) {
                integralSum += (error * dt);
            }
            else if (errorBoost == 0) {
                integralSum = 0;
            }
            integralSum = Math.max(-400, Math.min(400, integralSum));

            double rawDerivative = (error - lastError) / dt;
            double derivative = (0.8 * lastDerivative) + (0.2 * rawDerivative);
            lastDerivative = derivative;
            lastError = error;

            double power = (F * targetRPM) + (P * error) + (I * integralSum) + (D * derivative) + errorBoost;

            power = Math.max(0, Math.min(1.0, power));
            wheel.setPower(power);

            if (dataLog.size() < 5000) {
                dataLog.add(new String[]{
                        String.valueOf(runtime.seconds()),
                        String.valueOf(targetRPM),
                        String.valueOf(actualRPM),
                        String.valueOf(error),
                        String.valueOf(power),
                        String.valueOf(errorBoost)
                });
            }

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM", actualRPM);
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);
            telemetry.addData("Log Size", dataLog.size());
            telemetry.update();
        }

        wheel.setPower(0);
        saveCSV(dataLog);
    }

    private void saveCSV(ArrayList<String[]> data) {
        try {
            File path = new File(Environment.getExternalStorageDirectory().getAbsolutePath());
            File file = new File(path, "FPIDTChanges1.csv");

            CSVWriter writer = new CSVWriter(new FileWriter(file));
            writer.writeAll(data);
            writer.close();
            telemetry.addData("Status", "Saved to " + file.getAbsolutePath());
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("CSV Error", e.getMessage());
            telemetry.update();
        }
    }
}