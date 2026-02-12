package org.firstinspires.ftc.teamcode.shooter;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.shooter.wrappers.NewAxonServo;

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
    public static double P=0.07, I=0, D=0, F=0;
    public static double UPPER_HARD_STOP = 29;
    public static double LOWER_HARD_STOP = 13;
    public static double gearRatio = 1.0/12.0;


    Logger logger;
    Logger mechanismLogger;

    public PitchFSM(HWMap hwMap, Logger logger) {
        pitchServo = new NewAxonServo(hwMap.getPitchServo(),hwMap.getPitchEncoder(),false,false,0,gearRatio); // TODO: Change ratio
        state = States.ALIGNING;
        pidfController = new PIDFController(P,I,D,F);
        pidfController.setTolerance(TOLERANCE);
        this.logger = logger;
        this.mechanismLogger = mechanismLogger;
        targetAngle = 11;
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

        logger.log("error", error, Logger.LogLevels.DEBUG);

        double power = pidfController.calculate(pitchServo.getScaledPos(),targetAngle);
        logger.log("power", power, Logger.LogLevels.DEBUG);
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
        logger.log("pitch state", state, Logger.LogLevels.PRODUCTION);
        logger.log("pitch target angle", targetAngle, Logger.LogLevels.DEBUG);
        logger.log("pitch current angle", pitchServo.getScaledPos(), Logger.LogLevels.DEBUG);
    }
}
