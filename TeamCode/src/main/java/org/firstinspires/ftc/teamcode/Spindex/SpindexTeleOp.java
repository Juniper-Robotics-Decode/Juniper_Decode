package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;

@TeleOp
public class SpindexTeleOp extends LinearOpMode {
    private SpindexFSM spindexFSM;
    private HWMap hwMap;

        public SpindexTeleOp(HWMap hwMap, Telemetry telemetry) {
            spindexFSM = new SpindexFSM(hwMap, telemetry);
            this.telemetry = telemetry;
        }
@Override
    public void runOpMode(){

        waitForStart();
        while (opModeIsActive()){
            spindexFSM.updateState(hwMap,telemetry);
        }
    }

}
