package org.firstinspires.ftc.teamcode.intaketransfer;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.shooter.wrappers.NewAxonServo;

@Config
@TeleOp
public class transferServoPIDTest extends LinearOpMode {

    HWMap hwMap;
    private AxonServoWrapper transferServo;
    public static double targetAngle;


    public static double UPPER_HARD_STOP = 1;
    public static double LOWER_HARD_STOP = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap);
        transferServo = new AxonServoWrapper(hwMap.getTransferServo(),hwMap.getTransferEncoder(),false,false,0);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            updatePID();
            telemetry.addData("transfer target angle", targetAngle);
            telemetry.addData("transfer servo current angle", transferServo.getLastReadPos());
            telemetry.update();
        }
    }


    public void updatePID() {
        if(targetAngle > UPPER_HARD_STOP) {
            targetAngle = UPPER_HARD_STOP;
        }
        else if (targetAngle < LOWER_HARD_STOP) {
            targetAngle = LOWER_HARD_STOP;
        }
        transferServo.readPos();

        double error = targetAngle - transferServo.getLastReadPos();

        telemetry.addData("error", error);

        transferServo.setPos(targetAngle);
    }




}
