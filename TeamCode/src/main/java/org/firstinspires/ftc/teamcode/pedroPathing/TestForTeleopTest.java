package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestForTeleopTest extends LinearOpMode {
    TeleopTest teleopTest;

    @Override
    public void runOpMode() throws InterruptedException {
        teleopTest = new TeleopTest();
        teleopTest.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            teleopTest.drive(gamepad1,false,0);
        }
    }
}
