package org.firstinspires.ftc.teamcode.shooter.testClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

import java.io.FileWriter;
import java.io.IOException;

@Config
@TeleOp
public class TurretReset extends LinearOpMode {
    public DcMotorEx turretMotor;
    public double currentAMPS;
    public static double power = 0.1;

    FileWriter writer;

    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretMotor = hardwareMap.get(DcMotorEx.class, "TM");

        try {
            writer = new FileWriter("/sdcard/FIRST/turretCurrent+PosLog(P1).csv");
            writer.write("currentAMPS,encoderTicks\n");
        } catch (IOException e) {}

        waitForStart();

        while(opModeIsActive()){

            currentAMPS = turretMotor.getCurrent(CurrentUnit.AMPS);

            if(currentAMPS < 3.5){
                turretMotor.setPower(power);
            } else {
                turretMotor.setPower(0);
            }

            try {
                writer.write(currentAMPS + "," + turretMotor.getCurrentPosition() + "\n");
            } catch (IOException e) {}

            telemetry.addData("CurrentAMPS", currentAMPS);
            telemetry.addData("Motor Speed", turretMotor.getPower());
            telemetry.update();
        }

        try {
            writer.close();
        } catch (IOException e) {}
    }
}

/*
@TeleOp
public class TurretReset extends LinearOpMode {
    //motor moves @0.1 till voltage spikes, when that happens stop motor
    //make thresholds but if dont work thats fine
    //make new motor wrapper
    public MotorWrapper turretMotor;
    private HWMap HWMap;
    public double currentAMPS;
    public void runOpMode() {
        HWMap = new HWMap(hardwareMap);
        turretMotor = new MotorWrapper(HWMap.getTurretMotor(), false, 1, false);
        waitForStart();
        while(opModeIsActive()){
            currentAMPS = turretMotor.getCurrent();
            if(currentAMPS < 3.5){
                turretMotor.set(0.1);
            } else {
                turretMotor.set(0);
            }
            telemetry.addData("CurrentAMPS", currentAMPS);
            telemetry.addData("Motor Speed", turretMotor.get());
            telemetry.update();
        }
    }
}
*/