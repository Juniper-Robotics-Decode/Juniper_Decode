package org.firstinspires.ftc.teamcode.Swerve.Drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import static java.lang.Math.cos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.core.Logger;

@Config
public class SwerveModule {
    private final DcMotorEx motor;
    private final CRServo servo;
    private final AnalogInput encoder;
    private final Logger logger;

    private double offset;
    private boolean inverse;

    private double lastMotorPower;
    private double lastServoPower;
    private double lastTargetPosition;

    public static double P = 0.5, I = 0, D = 0.01, Kstatic = 0.02;
    private final PIDController rotationController = new PIDController(P, I, D);

    public SwerveModule(DcMotorEx motor, CRServo servo, AnalogInput encoder, double offset, boolean inverse, Logger logger) {
        this.motor = motor;
        this.servo = servo;
        this.encoder = encoder;
        this.offset = offset;
        this.inverse = inverse;
        this.logger = logger;

        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(double targetAngle, double drivePower) {
        double currentAngle = getCurrentRotation();
        double error = normalizeRadians(targetAngle - currentAngle);

        if (Math.abs(error) > Math.PI / 2) {
            drivePower *= -1;
            targetAngle = normalizeRadians(targetAngle + Math.PI);
        }

        rotationController.setPID(P, I, D);
        double steeringPower = rotationController.calculate(getCurrentRotation(), targetAngle);

        lastServoPower = Range.clip(steeringPower, -1, 1);
        lastMotorPower = drivePower;
        lastTargetPosition = targetAngle;

        servo.setPower(lastServoPower + ((error > 0.02) ? 0 : Kstatic));
        motor.setPower(lastMotorPower);
    }

    public double getCurrentRotation() {
        double pos = (encoder.getVoltage() / 3.3) * (2 * Math.PI);
        if (inverse) pos = (2 * Math.PI) - pos;
        return normalizeRadians(pos - offset);
    }

    public double getTrueX() { return getCurrentRotation(); }

    public double getMotorCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public void setOffset(double offset) { this.offset = offset; }

    public void setInverse(boolean inverse) { this.inverse = inverse; }

    public void setPID(double P, double I, double D){ rotationController.setPID(P,I,D); }

    public void log(int index) {
        if (logger != null) {
            String prefix = "Mod" + index + " ";

            logger.log(prefix + "Target", lastTargetPosition, Logger.LogLevels.PRODUCTION);
            logger.log(prefix + "Current", getCurrentRotation(), Logger.LogLevels.PRODUCTION);

            logger.log(prefix + "Servo Pwr", lastServoPower, Logger.LogLevels.DEBUG);
            logger.log(prefix + "Motor Pwr", lastMotorPower, Logger.LogLevels.DEBUG);
            logger.log(prefix + "Amps", motor.getCurrent(CurrentUnit.AMPS), Logger.LogLevels.DEBUG);
        }
    }
}