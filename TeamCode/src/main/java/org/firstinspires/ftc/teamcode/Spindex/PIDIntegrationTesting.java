package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Spindex.PIDS;

public class PIDIntegrationTesting extends LinearOpMode {
    private PIDS PIDS;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()){
            PIDS.spindex120Loop();
        }
    }
}
