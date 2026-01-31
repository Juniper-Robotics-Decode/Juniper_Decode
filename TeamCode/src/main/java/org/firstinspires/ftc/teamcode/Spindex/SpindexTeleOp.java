package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SpindexTeleOp extends LinearOpMode {

    public RevTouchSensor TCS;
    public DcMotor spindexMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        HWMapSpindex hwMap = new HWMapSpindex(hardwareMap);
        SpindexFSM spindexFSM = new SpindexFSM(hwMap, telemetry);
        TCS = hardwareMap.get(RevTouchSensor.class, "TCS");
        spindexMotor = hardwareMap.get(DcMotor.class, "spindexMotor");
        if(!TCS.isPressed()){
            spindexMotor.setPower(0.2);
        } else {
            spindexMotor.setPower(0);
            spindexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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