package org.firstinspires.ftc.teamcode.Spindex;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;

@TeleOp
@Config
public class SpindexTeleOp extends LinearOpMode {
    private SpindexFSM spindexFSM;
    private HWMap hwMap;
    public boolean shooting;
@Override
    public void runOpMode(){
        hwMap = new HWMap(hardwareMap);
        spindexFSM = new SpindexFSM(hwMap, telemetry);
        waitForStart();
        while (opModeIsActive()){
            spindexFSM.updateState(shooting);
        }
    }

}
