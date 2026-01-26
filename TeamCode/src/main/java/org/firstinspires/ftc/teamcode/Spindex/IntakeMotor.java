package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class IntakeMotor extends LinearOpMode {
    public DcMotor IntakeMotor;
    public static boolean intake;
    public static boolean FORCE_STOP;

    public int counter;
    public void runOpMode(){
        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake Motor");
        IntakeMotor.setPower(0);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while(opModeIsActive()) {
            if (intake) { //Detected Motif not full
                IntakeMotor.setPower(1);
            } else { //Detected Motif full
                IntakeMotor.setPower(-0.5);
            }
            if (FORCE_STOP && counter == 0) { //Gamepad intake button
                IntakeMotor.setPower(0);
                IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                counter++;
            }
            if (!FORCE_STOP && counter == 1){
                IntakeMotor.setPower(1);
                counter = 0;
            }
            telemetry.addData("Counter", counter);
            telemetry.addData("IntakeMotorPower", IntakeMotor.getPower());
            telemetry.addData("Intake?", intake);
        }
    }

}
