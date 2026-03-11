package org.firstinspires.ftc.teamcode.shooter.testClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.shooter.wrappers.NewAxonServo;
import org.firstinspires.ftc.teamcode.shooter.wrappers.LimelightCamera;
import org.firstinspires.ftc.teamcode.intake.IntakeFSM;
import org.firstinspires.ftc.teamcode.intaketransfer.TransferFSM;

@Config
@TeleOp
public class PitchFlywheelTuningWithTransfer extends LinearOpMode {

    TransferFSM transferFSM;
    IntakeFSM intakeFSM;
    MotorEx motor;

    HWMap hwMap;
    RobotSettings robotSettings;
    private NewAxonServo pitchServo;
    private LimelightCamera limelightCamera;
    public static double targetAngle;

    private PIDFController pidfController;
    public static double TOLERANCEPITCH = 1;
    public static double P=0.1, I=0, D=0, F=0;
    public static double gearRatio = 1.0/12.0;

    public static double UPPER_HARD_STOP = 25;
    public static double LOWER_HARD_STOP = 0;


    public static double vP=3, vI=0, vD=0, vF = 0;

    public static double ks=0, kv=1.7, ka=0;

    public static double defaultVelocity = 0;  // RPM

    public static double targetVelocityRPM = defaultVelocity;

    public static double targetVelocityTicks;

    private Logger logger;

    public static double TOLERANCE_FLYWHEEL = 100;

    public static double pitchReductionFactor = 0.1;

    public static double boostPower = 1;

    public static double pitchAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        logger = new Logger(telemetry);
        hwMap = new HWMap(hardwareMap);
        robotSettings = RobotSettings.load();

        transferFSM = new TransferFSM(hwMap, telemetry, logger);
        intakeFSM = new IntakeFSM(hwMap,telemetry, logger);
        motor = new MotorEx(hardwareMap,"FM", Motor.GoBILDA.BARE);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor.setRunMode(Motor.RunMode.VelocityControl);

        pitchServo = new NewAxonServo(hwMap.getPitchServo(),hwMap.getPitchEncoder(),false,false,0,gearRatio); // TODO: Change ratio
        limelightCamera = new LimelightCamera(hwMap.getLimelight(),telemetry, robotSettings);
        pidfController = new PIDFController(P,I,D,F);
        pidfController.setTolerance(TOLERANCEPITCH);

        waitForStart();
        while (opModeIsActive()) {
            limelightCamera.update();
            transferFSM.updateState(gamepad1.right_bumper);
            intakeFSM.updateState(gamepad1.y, gamepad1.dpad_left);
            updatePID();
            telemetry.addData("Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.addData("accel", motor.getAcceleration());

            updatePIDPitch();
            telemetry.addData("pitch target angle", targetAngle);
            telemetry.addData("pitch target angle corrected", pitchAngle);
            telemetry.addData("pitch servo current angle", pitchServo.getServoAngle());
            telemetry.addData("pitch current angle", pitchServo.getScaledPos());


            telemetry.addData("X", limelightCamera.getX());
            telemetry.addData("Y", limelightCamera.getY());
            telemetry.addData("Z", limelightCamera.getZ());
            telemetry.addData("Flat Distance", limelightCamera.getFlatDistance());
            telemetry.addData("tx",limelightCamera.getTx());
            telemetry.addData("ty", limelightCamera.getTy());
            telemetry.addData("Has target", limelightCamera.hasTarget());


            telemetry.update();
        }
    }



    public void updatePID() { // This method is used to update position every loop.

        /*pidfController.setPIDF(vP,vI,vD,vF);


        measuredVelocityTicks = motor.getCorrectedVelocity();
        //   measuredVelocityRPM = convertTicksToRPM(measuredVelocityTicks);

        // The error - sign (which finds velocity)
        double error = targetVelocity - measuredVelocityTicks;

        // We use zero because we already calculate for error
        double additionalPower = pidfController.calculate(error, 0);

        */


        motor.setVeloCoefficients(vP,vI,vD);
        motor.setFeedforwardCoefficients(ks,kv,ka);
        targetVelocityTicks = convertRPMToTicks(targetVelocityRPM);
        //targetVelocityTicks = targetVelocityTicks;
        double error = targetVelocityTicks - motor.getCorrectedVelocity();
        adjustForFlywheel(error);
        if(error > TOLERANCE_FLYWHEEL) {
            motor.set(boostPower);
        }
        else {
            motor.setVelocity(targetVelocityTicks);
        }
        telemetry.addData("Target Velocity RPM", targetVelocityRPM);
        telemetry.addData("Target Velocity Ticks", targetVelocityTicks);
        telemetry.addData("Current Velocity Corrected", motor.getCorrectedVelocity());
        telemetry.addData("Current Velocity Get", motor.getVelocity());
        telemetry.addData("flywheel error", error);
        telemetry.addData("flywheel power", motor.get());

        //motor.setVelocity(targetVelocity,RADIANS);
    }

    private static double convertRPMToTicks(double RPMVelocity) {
        return (RPMVelocity*28)/60;
    }


    public void updatePIDPitch() {
        if(pitchAngle > UPPER_HARD_STOP) {
            pitchAngle = UPPER_HARD_STOP;
        }
        else if (pitchAngle < LOWER_HARD_STOP) {
            pitchAngle = LOWER_HARD_STOP;
        }
        pidfController.setPIDF(P,I,D,F);
        pidfController.setTolerance(TOLERANCEPITCH);
        pitchServo.readPos();

        double error = pitchAngle - pitchServo.getScaledPos();

        telemetry.addData("error", error);

        double power = pidfController.calculate(pitchServo.getScaledPos(),pitchAngle);
        telemetry.addData("power", power);
        pitchServo.set(power);
    }

    private void adjustForFlywheel(double error) {
        double flywheelError = error;
        if(flywheelError > 100 && flywheelError < 1000) {
            double offset = flywheelError * pitchReductionFactor;
            pitchAngle = targetAngle + offset;
        } else {
            pitchAngle = targetAngle;
        }
    }

}
