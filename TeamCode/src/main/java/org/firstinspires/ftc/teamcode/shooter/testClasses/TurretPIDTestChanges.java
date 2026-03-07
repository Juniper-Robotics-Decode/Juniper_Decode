package org.firstinspires.ftc.teamcode.shooter.testClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

@Config
@TeleOp
public class TurretPIDTestChanges extends LinearOpMode {

    HWMap hwMap;
    MotorWrapper turretMotor;
    public static double targetAngle = 0;

    public static double P_Counter = 0.04;
    public static double P_clock = 0.02;
    public static double I = 0.0, D = 0, F = 0.2;

    private double lastError = 0;
    private double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();

    public static double TOLERANCE = 3;
    public static double gearRatio = 16.0/109.0;
    public static double UPPER_HARD_STOP = 0;
    public static double LOWER_HARD_STOP = -90;
    public static double POWER_CAP = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap);
        turretMotor = new MotorWrapper(hwMap.getTurretMotor(), false, gearRatio, false);
        turretMotor.resetEncoder();

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            updatePID();

            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", turretMotor.getScaledPos());
            telemetry.addData("Power", turretMotor.get());
            telemetry.update();
        }
    }

    public void updatePID() {
        targetAngle = Range.clip(targetAngle, LOWER_HARD_STOP, UPPER_HARD_STOP);

        turretMotor.readPosition();
        double currentPos = turretMotor.getScaledPos();
        double error = targetAngle - currentPos;
        double deltaTime = timer.seconds();
        if (deltaTime == 0) deltaTime = 0.001;
        timer.reset();

        double currentP = (error < 0) ? P_Counter : P_clock;

        if (Math.abs(error) < TOLERANCE * 2) {
            integralSum += error * deltaTime;
        } else {
            integralSum = 0;
        }

        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        double out = (error * currentP) + (integralSum * I) + (derivative * D);

        if (Math.abs(error) > TOLERANCE) {
            out += Math.signum(error) * F;
        } else {
            out = 0;
            integralSum = 0;
        }

        double finalPower = Range.clip(out, -POWER_CAP, POWER_CAP);

        if (currentPos >= UPPER_HARD_STOP && finalPower > 0) {
            finalPower = 0;
        } else if (currentPos <= LOWER_HARD_STOP && finalPower < 0) {
            finalPower = 0;
        }

        turretMotor.set(finalPower);

        telemetry.addData("Error", error);
        telemetry.addData("P Used", currentP);
    }
}