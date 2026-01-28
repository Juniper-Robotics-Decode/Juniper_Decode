package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;

@TeleOp
public class SpindexTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HWMapSpindex hwMap = new HWMapSpindex(hardwareMap);
        SpindexFSM spindexFSM = new SpindexFSM(hwMap, telemetry);

        waitForStart();
        while (opModeIsActive()) {
            int r = hwMap.getColorSensor1().red();
            int g = hwMap.getColorSensor1().green();
            int b = hwMap.getColorSensor1().blue();

            spindexFSM.updateState(getRuntime(), r, g, b);
            telemetry.update();
        }
    }
}
