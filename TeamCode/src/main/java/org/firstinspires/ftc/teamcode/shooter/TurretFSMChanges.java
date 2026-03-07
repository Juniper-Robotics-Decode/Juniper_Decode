package org.firstinspires.ftc.teamcode.shooter;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

@Config
public class TurretFSMChanges {
    public enum States {
        ALIGNING,
        ALIGNED
    }

    private MotorWrapper turretMotor;
    private States state;
    private double targetAngle;


    public static double P = 0.015, I = 0.08, D = 0.0001, F = 0.15;
    public static double TOLERANCE = 2.0;
    public static double INTEGRAL_LIMIT = 0.2;
    public static double POWER_CAP = 1.0;

    private double lastError = 0;
    private double integralSum = 0;
    private double lastUpdateTime = 0;

    public static double gearRatio = 16.0 / 109.0;
    public static double TURRET_OFFSET = 3;
    private double MANUAL_OFFSET = 0;

    public boolean isCalibrating = false;
    private int i = 0;

    private Telemetry telemetry;
    private Logger logger;

    private boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

    public TurretFSMChanges(HWMap hwMap, Telemetry telemetry, Logger logger) {
        this.logger = logger;
        this.telemetry = telemetry;

        turretMotor = new MotorWrapper(hwMap.getTurretMotor(), false, gearRatio, false);
        turretMotor.resetEncoder();
        state = States.ALIGNING;
        lastUpdateTime = System.nanoTime() / 1E9;
    }

    public void updateState() {
        updatePID();

        double currentPos = normalizeDegrees(turretMotor.getScaledPos());
        double error = normalizeDegrees(targetAngle - currentPos);

        state = (Math.abs(error) < TOLERANCE) ? States.ALIGNED : States.ALIGNING;
    }

    public void updatePID() {
        turretMotor.readPosition();
        double currentTime = System.nanoTime() / 1E9;
        double deltaTime = currentTime - lastUpdateTime;

        double currentPos = normalizeDegrees(turretMotor.getScaledPos());
        double normalizedTarget = normalizeDegrees(targetAngle);
        double error = normalizeDegrees(normalizedTarget - currentPos);

        if (Math.abs(error) < TOLERANCE * 3) {
            integralSum += error * deltaTime;
        } else {
            integralSum = 0;
        }

        double iTerm = I * integralSum;
        iTerm = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, iTerm));

        double derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;

        double power = (P * error) + iTerm + (D * derivative);

        if (Math.abs(error) > 0.5) {
            power += (F * Math.signum(error));
        }

        if (Math.abs(power) > POWER_CAP) {
            power = Math.signum(power) * POWER_CAP;
        }

        turretMotor.set(power);

        lastError = error;
        lastUpdateTime = currentTime;

        telemetry.addData("Turret Error", error);
        telemetry.addData("Turret Power", power);
        telemetry.addData("Turret Deg", currentPos);
    }

    public void setTargetAngle(double turretError, boolean dPadUp2, boolean dPadDown2, boolean dPadLeft2, boolean dPadRight2, boolean leftBumper2) {
        if (leftBumper2) isCalibrating = !isCalibrating;

        if (dPadUp2 && !lastUp) MANUAL_OFFSET += 10;
        if (dPadDown2 && !lastDown) MANUAL_OFFSET -= 10;
        if (dPadRight2 && !lastRight) MANUAL_OFFSET += 1;
        if (dPadLeft2 && !lastLeft) MANUAL_OFFSET -= 1;

        lastUp = dPadUp2; lastDown = dPadDown2; lastLeft = dPadLeft2; lastRight = dPadRight2;

        if (PositionFSM.sensor == PositionFSM.Sensor.PINPOINT) {
            targetAngle = -turretError + MANUAL_OFFSET;
        } else {
            if (i >= 50) {
                targetAngle = turretMotor.getScaledPos() + turretError + TURRET_OFFSET + MANUAL_OFFSET;
                i = 0;
            } else {
                i++;
            }
        }
    }

    public boolean ALIGNED() {
        return state == States.ALIGNED;
    }

    public void log() {
        logger.log("---Turret---", "", Logger.LogLevels.PRODUCTION);
        logger.log("Target Angle", targetAngle, Logger.LogLevels.PRODUCTION);
        logger.log("Current Angle", normalizeDegrees(turretMotor.getScaledPos()), Logger.LogLevels.PRODUCTION);
        logger.log("Error", lastError, Logger.LogLevels.DEBUG);
    }

    public void resetTurret() {
        turretMotor.resetEncoder();
        MANUAL_OFFSET = 0;
        targetAngle = 0;
        integralSum = 0;
    }
}