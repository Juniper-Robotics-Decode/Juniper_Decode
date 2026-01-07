package org.firstinspires.ftc.teamcode.shooter.testClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class FlywheelPIDTest extends LinearOpMode {

    MotorEx motor1;
    public static double vP=3, vI=0, vD=0, vF = 0;

    public static double ks=0, kv=2, ka=0;

    public static double defaultVelocity = 0;  // RPM

    public static double targetVelocityRPM = defaultVelocity;

    public static double targetVelocityTicks;

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = new MotorEx(hardwareMap,"FM", Motor.GoBILDA.BARE);
        motor1.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor1.setRunMode(Motor.RunMode.VelocityControl);

        waitForStart();
        while (opModeIsActive()) {
            updatePID();
            telemetry.addData("Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.addData("accel", motor1.getAcceleration());
            telemetry.update();
        }
    }



    public void updatePID() { // This method is used to update position every loop.

        /*pidfController.setPIDF(vP,vI,vD,vF);


        measuredVelocityTicks = motor.getCorrectedVelocity();
        //   measuredVelocityRPM = convertTicksToRPM(measuredVelocityTicks);

        // The error - sign (which finds velocity)
        double error = targetVelocity - measuredVelocityTicks;

        // We use zero because we already calculate for error
        double additionalPower = pidfController.calculate(error, 0);

        */


        motor1.setVeloCoefficients(vP,vI,vD);
        motor1.setFeedforwardCoefficients(ks,kv,ka);
        targetVelocityTicks = convertRPMToTicks(targetVelocityRPM);
        targetVelocityTicks = -targetVelocityTicks;
        motor1.setVelocity(targetVelocityTicks);
        telemetry.addData("Target Velocity RPM", targetVelocityRPM);
        telemetry.addData("Target Velocity Ticks", targetVelocityTicks);
        telemetry.addData("Current Velocity Corrected", motor1.getCorrectedVelocity());
        telemetry.addData("Current Velocity Get", motor1.getVelocity());

        //motor.setVelocity(targetVelocity,RADIANS);
    }
    private static double convertRPMToTicks(double RPMVelocity) {
        return (RPMVelocity*28)/60;
    }



}
