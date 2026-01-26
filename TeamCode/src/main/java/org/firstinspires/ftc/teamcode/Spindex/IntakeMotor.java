package org.firstinspires.ftc.teamcode.Spindex;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Config
public class IntakeMotor extends LinearOpMode {
    public DcMotor IntakeMotor;
    public static boolean intake = true;
    public static boolean FORCE_STOP = false;

    public void runOpMode(){
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        IntakeMotor.setPower(0);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while(opModeIsActive()) {
            if (intake && !FORCE_STOP) { //Detected Motif not full
                IntakeMotor.setPower(1);
            } else if(!intake && !FORCE_STOP){ //Detected Motif full
                IntakeMotor.setPower(-0.5);
            } else if (FORCE_STOP){ //stop requested
                IntakeMotor.setPower(0);

            }
//1
            telemetry.addData("IntakeMotorPower", IntakeMotor.getPower());
            telemetry.addData("Intake?", intake);
            telemetry.addData("FORCE STOP?", FORCE_STOP);
            telemetry.update();
        }
    }
}
