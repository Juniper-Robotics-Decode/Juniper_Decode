package org.firstinspires.ftc.teamcode.Spindex;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DigitalChannel;
//1
@TeleOp
@Config
public class analogInput extends LinearOpMode {
    public DigitalChannel AI;
    public DcMotor motor;
    public static double threshold = 1.0;
    public boolean pressed;

    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        AI = hardwareMap.get(DigitalChannel.class, "AI");
        motor = hardwareMap.get(DcMotor.class,"motor");
        waitForStart();
        while(opModeIsActive()){
            pressed = AI.getState(); //true = state
            if (pressed) {
                telemetry.addData("Input Status", "PRESSED");
                motor.setPower(0);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                telemetry.addData("Input Status", "NOT PRESSED");
                motor.setPower(0.5);
            }
            telemetry.addData("State", AI.getState());
            telemetry.update();

        }

    }
}
