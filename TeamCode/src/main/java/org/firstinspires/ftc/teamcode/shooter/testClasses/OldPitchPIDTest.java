package org.firstinspires.ftc.teamcode.shooter.testClasses;



import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.shooter.wrappers.AxonCRServoWrapper;
import org.firstinspires.ftc.teamcode.core.HWMap;

@Config
@TeleOp(name = "", group = "2")
public class OldPitchPIDTest extends LinearOpMode {

    HWMap hwMap;
    private AxonCRServoWrapper pitchServo;
    public static double targetAngle;

    private PIDFController pidfController;
    public static double TOLERANCE = 1;
    public static double P=0.1, I=0, D=0, F=0;
    public static double gearRatio = 1.0/12.0;

    public static double UPPER_HARD_STOP = 29;
    public static double LOWER_HARD_STOP = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap);
        pitchServo = new AxonCRServoWrapper(hwMap.getPitchServo(),hwMap.getPitchEncoder(),false,false,0,gearRatio); // TODO: Change ratio
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pidfController = new PIDFController(P,I,D,F);
        pidfController.setTolerance(TOLERANCE);

        waitForStart();
        while (opModeIsActive()) {
            updatePID();
            telemetry.addData("pitch target angle", targetAngle);
            telemetry.addData("pitch servo current angle", pitchServo.getServoAngle());
            telemetry.addData("pitch current angle", pitchServo.getScaledPos());
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
        pidfController.setPIDF(P,I,D,F);
        pidfController.setTolerance(TOLERANCE);
        pitchServo.readPos();

        double delta = angleDelta(pitchServo.getScaledPos(), targetAngle);
        double sign = angleDeltaSign(pitchServo.getScaledPos(), targetAngle);
        double error = delta * sign;

        double power = pidfController.calculate(error,0);
        pitchServo.set(power);
    }


    private double angleDelta(double measuredAngle, double targetAngle) {
        return Math.min(normalizeDegrees(measuredAngle - targetAngle), 360 - normalizeDegrees(measuredAngle - targetAngle));
    }

    private double angleDeltaSign(double measuredAngle, double targetAngle) {
        return -(Math.signum(normalizeDegrees(targetAngle - measuredAngle) - (360 - normalizeDegrees(targetAngle - measuredAngle))));
    }



}


