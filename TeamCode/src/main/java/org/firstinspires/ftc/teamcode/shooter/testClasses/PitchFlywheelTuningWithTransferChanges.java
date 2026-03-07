package org.firstinspires.ftc.teamcode.shooter.testClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.shooter.wrappers.NewAxonServo;
import org.firstinspires.ftc.teamcode.shooter.wrappers.LimelightCamera;
import org.firstinspires.ftc.teamcode.intake.IntakeFSM;
import org.firstinspires.ftc.teamcode.intaketransfer.TransferFSM;

@Config
@TeleOp
public class PitchFlywheelTuningWithTransferChanges extends LinearOpMode {

    TransferFSM transferFSM;
    IntakeFSM intakeFSM;
    MotorEx motor;

    HWMap hwMap;
    RobotSettings robotSettings;
    private NewAxonServo pitchServo;
    private LimelightCamera limelightCamera;

    public static double targetAngle;
    private PIDFController pitchPIDF;
    public static double TOLERANCE = 1;
    public static double P=0.1, I=0, D=0, F=0;
    public static double gearRatio = 1.0/12.0;
    public static double UPPER_HARD_STOP = 28;
    public static double LOWER_HARD_STOP = 10;

    public static double vP = 0.12;
    public static double vI = 0.003;
    public static double vD = 0.08;
    public static double vF = 0.00015;

    public static double errorBoostThreshold = 80;
    public static double errorBoostPower = 0.45;
    public static double targetRPM = 2650;

    private double integralSum = 0;
    private double lastError = 0;
    private double lastDerivative = 0;
    private ElapsedTime timer = new ElapsedTime();
    private Logger logger;

    @Override
    public void runOpMode() throws InterruptedException {
        logger = new Logger(telemetry);
        hwMap = new HWMap(hardwareMap);
        robotSettings = RobotSettings.load();

        transferFSM = new TransferFSM(hwMap, telemetry, logger);
        intakeFSM = new IntakeFSM(hwMap, telemetry, transferFSM, logger);

        motor = new MotorEx(hardwareMap, "FM", Motor.GoBILDA.BARE);
        motor.setInverted(true);
        motor.setRunMode(Motor.RunMode.RawPower);

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pitchServo = new NewAxonServo(hwMap.getPitchServo(), hwMap.getPitchEncoder(), false, false, 0, gearRatio);
        pitchPIDF = new PIDFController(P, I, D, F);

        timer.reset();
        waitForStart();

        while (opModeIsActive()) {
            if (limelightCamera != null) limelightCamera.update();
            transferFSM.updateState(gamepad1.right_bumper);
            intakeFSM.updateState(gamepad1.y, gamepad1.dpad_left);

            updateFlywheelPID();
            updatePIDPitch();

            telemetry.addData("Flywheel RPM", getCurrentRPM());
            telemetry.addData("Flywheel Target", targetRPM);
            telemetry.addData("Pitch Angle", pitchServo.getScaledPos());
            telemetry.addData("Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.update();
        }
    }

    public void updateFlywheelPID() {
        double dt = timer.seconds();
        if (dt <= 0) dt = 0.001;
        timer.reset();

        double currentRPM = getCurrentRPM();
        double error = targetRPM - currentRPM;

        double errorBoost = 0;
        if (error > errorBoostThreshold) {
            errorBoost = errorBoostPower;
            integralSum = 0;
        }

        if (Math.abs(error) < 150) {
            integralSum += (error * dt);
        } else {
            integralSum = 0;
        }
        integralSum = Math.max(-20, Math.min(20, integralSum));

        double rawDerivative = (error - lastError) / dt;
        double derivative = (0.8 * lastDerivative) + (0.2 * rawDerivative);
        lastError = error;
        lastDerivative = derivative;

        double power = (vF * targetRPM) + (vP * error) + (vI * integralSum) + (vD * derivative) + errorBoost;

        power = Math.max(0, Math.min(1.0, power));
        motor.set(power);

        telemetry.addData("Flywheel Power", power);
        telemetry.addData("Boost Active", errorBoost > 0);
    }

    private double getCurrentRPM() {
        return (motor.getCorrectedVelocity() / 28.0) * 60.0;
    }

    public void updatePIDPitch() {
        targetAngle = Math.max(LOWER_HARD_STOP, Math.min(UPPER_HARD_STOP, targetAngle));

        pitchPIDF.setPIDF(P, I, D, F);
        pitchPIDF.setTolerance(TOLERANCE);
        pitchServo.readPos();

        double power = pitchPIDF.calculate(pitchServo.getScaledPos(), targetAngle);
        pitchServo.set(power);
    }
}