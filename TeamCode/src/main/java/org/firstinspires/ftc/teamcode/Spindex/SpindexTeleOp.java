package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;

@TeleOp
public class SpindexTeleOp extends LinearOpMode {
    private SpindexFSM spindexFSM;
    private HWMap hwMap;


@Override
    public void runOpMode(){
        hwMap = new HWMap(hardwareMap);
        spindexFSM = new SpindexFSM(hwMap, telemetry);
        waitForStart();
        while (opModeIsActive()){
            spindexFSM.updateState(hwMap,telemetry);
        }
    }

}
