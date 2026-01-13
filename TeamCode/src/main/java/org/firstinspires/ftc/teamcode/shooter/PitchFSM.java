package org.firstinspires.ftc.teamcode.shooter;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.shooter.wrappers.NewAxonServo;

import java.util.function.DoubleSupplier;

@Config
public class PitchFSM {
    public enum States{
        ALIGNING,
        ALIGNED
    }

    private NewAxonServo pitchServo;
    private States state;
    private double targetAngle;

    private PIDFController pidfController;
    public static double TOLERANCE = 1;
    public static double P=0.05, I=0, D=0, F=0;
    public static double UPPER_HARD_STOP = 29;
    public static double LOWER_HARD_STOP = 10;
    public static double gearRatio = 1.0/12.0;
    public static double pitchReductionFactor = 0.10;
    private DoubleSupplier flywheelErrorProvider;


    Telemetry telemetry;

    public PitchFSM(HWMap hwMap, Telemetry telemetry, DoubleSupplier flywheelErrorProvider) {
        pitchServo = new NewAxonServo(hwMap.getPitchServo(),hwMap.getPitchEncoder(),false,false,0,gearRatio); // TODO: Change ratio
        state = States.ALIGNING;
        pidfController = new PIDFController(P,I,D,F);
        pidfController.setTolerance(TOLERANCE);
        this.telemetry = telemetry;
        targetAngle = 11;
        this.flywheelErrorProvider = flywheelErrorProvider;
    }

    public void updateState(){
        updatePID();
        if(pidfController.atSetPoint()) {
            state = States.ALIGNED;
        }
        else {
            state = States.ALIGNING;
        }
    }

    public void updatePID() {
      //  adjustForFlywheel();
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

/* uses rotational logic
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
    */

    public void setTargetAngle(double pitchTargetAngle) {
        targetAngle = pitchTargetAngle;
    }

    public boolean ALIGNED() {
        return state == States.ALIGNED;
    }

    public void log() {
        telemetry.addData("pitch state", state);
        telemetry.addData("pitch target angle", targetAngle);
        telemetry.addData("pitch current angle", pitchServo.getScaledPos());
    }

    private void adjustForFlywheel() {
        double flywheelError = flywheelErrorProvider.getAsDouble();
        if(flywheelError > 40) {
            targetAngle = targetAngle + flywheelError * pitchReductionFactor;
        }
    }

}
