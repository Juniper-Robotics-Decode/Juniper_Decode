package org.firstinspires.ftc.teamcode.Spindex;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
@Config
public class analogInput extends LinearOpMode {
    public DigitalChannel LimitSwitch;
    public boolean pressed;
    public DcMotor spindexMotor;
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        LimitSwitch = hardwareMap.get(DigitalChannel.class, "LimitSwitch");
        spindexMotor = hardwareMap.get(DcMotor.class, "spindexMotor");
        waitForStart();

        while (opModeIsActive()) {
            pressed = LimitSwitch.getState();
            if(pressed){
                spindexMotor.setPower(0);
            } else{
                spindexMotor.setPower(1);
                spindexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            telemetry.addData("SpindexMotor Power", spindexMotor.getPower());
            telemetry.addData("State", pressed);
            telemetry.update();
        }
    }
}