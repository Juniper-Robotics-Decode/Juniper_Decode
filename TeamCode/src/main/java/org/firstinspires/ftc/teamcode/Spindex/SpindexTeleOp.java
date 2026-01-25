package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

@TeleOp
public class SpindexTeleOp extends LinearOpMode {
    public DigitalChannel AI;
    public MotorWrapper spindexMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        HWMapSpindex hwMap = new HWMapSpindex(hardwareMap);
        SpindexFSM spindexFSM = new SpindexFSM(hwMap, telemetry);
        spindexMotor = new MotorWrapper(hwMap.getSpindexMotor(),false,1, 537.7);
        AI = hwMap.getAnalogInput();
        while (!AI.getState()){
            spindexMotor.set(0.1);
            telemetry.addData("Motor Position", "Going to Origin");
            telemetry.update();
        }
        while (AI.getState()){
            spindexMotor.set(0);
            spindexMotor.brakeModeMotor();
            spindexMotor.resetEncoder();
            telemetry.addData("Motor Position", "Set to Origin");
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            boolean shooting = gamepad1.right_bumper;
            int r = hwMap.getColorSensor1().red();
            int g = hwMap.getColorSensor1().green();
            int b = hwMap.getColorSensor1().blue();

            spindexFSM.updateState(shooting, getRuntime(), r, g, b);
            telemetry.update();
        }
    }
}