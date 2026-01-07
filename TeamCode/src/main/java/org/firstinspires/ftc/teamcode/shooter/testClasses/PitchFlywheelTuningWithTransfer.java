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
    public static double TOLERANCE = 1;
    public static double P=0.1, I=0, D=0, F=0;
    public static double gearRatio = 1.0/12.0;

    public static double UPPER_HARD_STOP = 28;
    public static double LOWER_HARD_STOP = 10;


    public static double vP=3, vI=0, vD=0, vF = 0;

    public static double ks=0, kv=2, ka=0;

    public static double defaultVelocity = 0;  // RPM

    public static double targetVelocityRPM = defaultVelocity;

    public static double targetVelocityTicks;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HWMap(hardwareMap);
        robotSettings = RobotSettings.load();
        intakeFSM = new IntakeFSM(hwMap,telemetry);
        transferFSM = new TransferFSM(hwMap, telemetry);
        motor = new MotorEx(hardwareMap,"FM", Motor.GoBILDA.BARE);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor.setRunMode(Motor.RunMode.VelocityControl);

        pitchServo = new NewAxonServo(hwMap.getPitchServo(),hwMap.getPitchEncoder(),false,false,0,gearRatio); // TODO: Change ratio
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelightCamera = new LimelightCamera(hwMap.getLimelight(),telemetry, robotSettings);
        pidfController = new PIDFController(P,I,D,F);
        pidfController.setTolerance(TOLERANCE);

        waitForStart();
        while (opModeIsActive()) {
            limelightCamera.update();
            transferFSM.updateState(gamepad1.dpad_right, gamepad1.right_bumper);
            intakeFSM.updateState(gamepad1.y, gamepad1.dpad_left);
            updatePID();
            telemetry.addData("Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());
            telemetry.addData("accel", motor.getAcceleration());

            updatePIDPitch();
            telemetry.addData("pitch target angle", targetAngle);
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
        targetVelocityTicks = -targetVelocityTicks;
        motor.setVelocity(targetVelocityTicks);
        telemetry.addData("Target Velocity RPM", targetVelocityRPM);
        telemetry.addData("Target Velocity Ticks", targetVelocityTicks);
        telemetry.addData("Current Velocity Corrected", motor.getCorrectedVelocity());
        telemetry.addData("Current Velocity Get", motor.getVelocity());

        //motor.setVelocity(targetVelocity,RADIANS);
    }
    private static double convertRPMToTicks(double RPMVelocity) {
        return (RPMVelocity*28)/60;
    }


    public void updatePIDPitch() {
        if(targetAngle > UPPER_HARD_STOP) {
            targetAngle = UPPER_HARD_STOP;
        }
        else if (targetAngle < LOWER_HARD_STOP) {
            targetAngle = LOWER_HARD_STOP;
        }
        pidfController.setPIDF(P,I,D,F);
        pidfController.setTolerance(TOLERANCE);
        pitchServo.readPos();

        double error = targetAngle - pitchServo.getScaledPos();

        telemetry.addData("error", error);

        double power = pidfController.calculate(pitchServo.getScaledPos(),targetAngle);
        telemetry.addData("power", power);
        pitchServo.set(power);
    }

}
