package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class Encoder_Test extends LinearOpMode {

    public DcMotor EncoderTest;
    int target = 50;
    @Override
    public void runOpMode() {
         EncoderTest = hardwareMap.get(DcMotor.class, "spindexMotor");



        EncoderTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

 //       EncoderTest.setTargetPosition(target);
  //      EncoderTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Init & Ready");
        telemetry.update();

        waitForStart();

  //      EncoderTest.setPower(0.5);

        while (opModeIsActive()) {
            telemetry.addData("Target:", target);
            telemetry.addData("Current Position:", EncoderTest.getCurrentPosition());
            telemetry.update();
        }

        EncoderTest.setPower(0);

        telemetry.addData("Status", "Rotation Complete");
        telemetry.update();
    }
}
