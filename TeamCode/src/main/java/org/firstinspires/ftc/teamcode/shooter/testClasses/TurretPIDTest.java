package org.firstinspires.ftc.teamcode.shooter.testClasses;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

import java.util.concurrent.TimeUnit;


@Config
@TeleOp
public class TurretPIDTest extends LinearOpMode {

    HWMap hwMap;
    MotorWrapper turretMotor;
    public static double targetAngle;

    private PIDController pidController;
    public static double TOLERANCE = 3;
    public static double P_Counter = 0.04;
    public static double P_clock =  0.02;
    public static double P=0.01, I=0.0, D=0, F=0.2;
    public static double gearRatio = 16.0/109.0;

    public static double UPPER_HARD_STOP = 0;
    public static double LOWER_HARD_STOP = -90;

    public static double POWER_CAP = 1;


    Timing.Timer timer;
    public static long sleepTime = 45;

    @Override
    public void runOpMode() throws InterruptedException {

        timer = new Timing.Timer(3000000, TimeUnit.MILLISECONDS);
        hwMap = new HWMap(hardwareMap);
        turretMotor = new MotorWrapper(hwMap.getTurretMotor(),false,gearRatio, false);
        turretMotor.resetEncoder();
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pidController = new PIDController(P,I,D);

        waitForStart();
        while (opModeIsActive()) {
            timer.start();
            updatePID();
            telemetry.addData("turret target angle", targetAngle);
            telemetry.addData("Turret motor target Angle", targetAngle * (109.0/16.0));
            telemetry.addData("turret motor current angle", turretMotor.getAngle());
            telemetry.addData("turret current angle", turretMotor.getScaledPos());
            telemetry.addData("TIcks per rev", turretMotor.getTicksPerRev());
            telemetry.addData("Curret Pos ticks", turretMotor.readandGetPosTicks());
            telemetry.addData("loop time", timer.elapsedTime());
            telemetry.update();
            sleep(sleepTime);
        }
    }



    public void updatePID() {

        pidController.setPID(P,I,D);
        pidController.setTolerance(TOLERANCE);
        turretMotor.readPosition();
        if(targetAngle > UPPER_HARD_STOP) {
            targetAngle = UPPER_HARD_STOP;
        }
        else if (targetAngle < LOWER_HARD_STOP) {
            targetAngle = LOWER_HARD_STOP;
        }

/*
        double delta = angleDelta(turretMotor.getScaledPos(), targetAngle);
        double sign = angleDeltaSign(turretMotor.getScaledPos(), targetAngle);*/
        double currentPos = turretMotor.getScaledPos();
        double error = targetAngle - currentPos;
        telemetry.addData("Error", error);

        if (error < 0) {
            P  = P_Counter;
        }
        else {
            P = P_clock;
        }
        double power = pidController.calculate(currentPos,targetAngle);
        if(Math.abs(error) >= TOLERANCE) {
            power = power + (F * Math.signum(error));
        }
        if(Math.abs(power) > POWER_CAP) {
            double signPower = Math.signum(power);
            power = signPower*POWER_CAP;
        }
        turretMotor.set(power);
        telemetry.addData("power", turretMotor.get());
    }




}

