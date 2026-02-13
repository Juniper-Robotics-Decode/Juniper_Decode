package org.firstinspires.ftc.teamcode.shooter;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
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
    private double MANUAL_OFFSET = 0;
    private DoubleSupplier flywheelErrorProvider;


    Telemetry telemetry;

    Logger logger;

    public PitchFSM(HWMap hwMap, Telemetry telemetry, DoubleSupplier flywheelErrorProvider, Logger logger) {
        this.logger = logger;
        pitchServo = new NewAxonServo(hwMap.getPitchServo(),hwMap.getPitchEncoder(),false,false,0,gearRatio); // TODO: Change ratio
        state = States.ALIGNING;
        pidfController = new PIDFController(P,I,D,F);
        pidfController.setTolerance(TOLERANCE);
        this.telemetry = telemetry;
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        targetAngle = 11;
        this.flywheelErrorProvider = flywheelErrorProvider;
    }

    public void updateState(boolean yPress2, boolean aPress2){
        updatePID(yPress2,aPress2);
        if(pidfController.atSetPoint()) {
            state = States.ALIGNED;
        }
        else {
            state = States.ALIGNING;
        }
    }

    public void updatePID(boolean yPress, boolean aPress) {
        if(yPress) {
            MANUAL_OFFSET--;
        }
        if(aPress) {
            MANUAL_OFFSET++;
        }


        targetAngle = targetAngle + MANUAL_OFFSET;
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

//        telemetry.addData("error", error);

        double power = pidfController.calculate(pitchServo.getScaledPos(),targetAngle);
//        telemetry.addData("power", power);
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
        logger.log("<font color='yellow'>-----------Pitch-------</font>", "", Logger.LogLevels.PRODUCTION);
        logger.log("<b><font color='green'>pitch Manual offset</font></b>", MANUAL_OFFSET, Logger.LogLevels.PRODUCTION);
        logger.log("pitch state", state, Logger.LogLevels.DEBUG);
        logger.log("pitch target angle", targetAngle, Logger.LogLevels.PRODUCTION);
        logger.log("pitch current angle", pitchServo.getScaledPos(), Logger.LogLevels.PRODUCTION);
    }

    private void adjustForFlywheel() {
        double flywheelError = flywheelErrorProvider.getAsDouble();
        if(flywheelError > 40) {
            targetAngle = targetAngle + flywheelError * pitchReductionFactor;
        }
    }

}
