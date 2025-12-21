
package org.firstinspires.ftc.teamcode.core;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MotorWrapper {
    private final MotorEx motorEx;
    private double lastReadVelocity;
    private double lastReadPosition;
    private double ratio;
    private final double TICK_PER_REVOLUTION;

    public MotorWrapper(MotorEx motorEx, boolean velocityControl, double ratio) {
        this.motorEx = motorEx;
        if (velocityControl) {
            motorEx.setRunMode(Motor.RunMode.VelocityControl);
        }

        this.ratio = ratio;
        TICK_PER_REVOLUTION = 145.1;
    }

    // POWER

/**
     Description: The set method is a wrapper of the motor set method
     @param : the power to set the motor at
     */

    public void set(double power) {
        motorEx.set(power);
    }

    public double get() {
        return motorEx.get();
    }

    // VELOCITY
/**
     Description: The following method reads the encoder to get velocity of the motor
     @return: the velocity that is just read
     */

    public void readVelocity() {
        lastReadVelocity = motorEx.getCorrectedVelocity();
    }


/**
     Description: The following method gets the velocity of the motor that was last read
     @return: the velocity that was read before
     */

    public double getVelocity() {
        return lastReadVelocity;
    }

    public void setVelocity(double targetVelocity) {
        motorEx.setVelocity(targetVelocity);
    }

    public void setVelocityConstants(double vP, double vI, double vD, double ks, double kv, double ka) {
        motorEx.setVeloCoefficients(vP,vI,vD);
        motorEx.setFeedforwardCoefficients(ks,kv,ka);

    }

    // POSITION

    public void readPosition() {
        lastReadPosition = (motorEx.getCurrentPosition()/TICK_PER_REVOLUTION)*360;
    }

    public double getTicksPerRev() {
        return TICK_PER_REVOLUTION;
    }

    public double readandGetPosTicks() {
        return motorEx.getCurrentPosition();
    }

    public double getAngle() {
        return lastReadPosition;
    }

    public double getScaledPos() {
        return lastReadPosition*ratio;
    }

    public void resetEncoder() {
        motorEx.stopAndResetEncoder();
    }

    public double getCurrent() {
        return motorEx.motorEx.getCurrent(CurrentUnit.AMPS);
    }
}

