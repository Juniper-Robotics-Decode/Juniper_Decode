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
    public AnalogInput LimitSwitch;
    public DcMotor motor;
    public boolean pressed;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LimitSwitch = hardwareMap.get(AnalogInput.class, "LimitSwitch");
        motor = hardwareMap.get(DcMotor.class, "spindexMotor");

        waitForStart();

        while (opModeIsActive()) {
            double currentVoltage = LimitSwitch.getVoltage();
       //     pressed = LimitSwitch.volta(); //true = state
       /*     if (pressed) {
                motor.setPower(0);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                motor.setPower(0.5);
            }
            telemetry.addData("State", LimitSwitch.getState());
            telemetry.addData("Pressed?", pressed);

            telemetry.update();

        */
            telemetry.addData("Voltage", currentVoltage);
            telemetry.update();
        }
    }
}