package org.firstinspires.ftc.teamcode.Spindex;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.core.HWMapSpindex;


@TeleOp
public class SpindexTeleOp extends LinearOpMode {
    public DigitalChannel LimitSwitch;
    @Override
    public void runOpMode() throws InterruptedException {
        HWMapSpindex hwMap = new HWMapSpindex(hardwareMap);
        SpindexFSM spindexFSM = new SpindexFSM(hwMap, telemetry);
        LimitSwitch = hwMap.getLimiSwitch();
        while (!isStarted() && !isStopRequested()) {
            if (spindexFSM.homeSpindex()) {
                telemetry.addData("LimitSwitch", LimitSwitch.getState());
                telemetry.update();
                break; // Stop moving once the switch is hit
            }
        }
        waitForStart();
        while (opModeIsActive()) {
            int r = hwMap.getColorSensor1().red();
            int g = hwMap.getColorSensor1().green();
            int b = hwMap.getColorSensor1().blue();

            spindexFSM.updateState(getRuntime(), r, g, b, gamepad1);
            telemetry.update();
        }
    }
}