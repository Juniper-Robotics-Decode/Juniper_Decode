package org.firstinspires.ftc.teamcode.Spindex;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;

@TeleOp
@Config
public class SpindexTeleOp extends LinearOpMode {
    private SpindexFSM spindexFSM;
    private HWMapSpindex hwMap;
    public static boolean shooting = false;
    @Override
    public void runOpMode(){
        hwMap = new HWMapSpindex(hardwareMap);
        spindexFSM = new SpindexFSM(hwMap, telemetry);
        waitForStart();
        while (opModeIsActive()){
            shooting = gamepad1.right_bumper;
            spindexFSM.updateState(shooting);
            telemetry.update();
        }
    }

}