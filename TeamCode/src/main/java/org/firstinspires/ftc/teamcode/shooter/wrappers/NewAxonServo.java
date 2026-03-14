package org.firstinspires.ftc.teamcode.shooter.wrappers;


import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class NewAxonServo {
    private CRServo axon;
    private AnalogInput encoder;
    private double sign = 1;
    private double ratio;

    private double continuousServoAngle = 0;
    private double lastRawServoAngle = 0;
    private double encoderOffset = 0;
    private boolean isFirstRead = true;

    private boolean inverseEncoder = false;


    public NewAxonServo(CRServo axon, AnalogInput encoder, boolean inversePower, boolean inverseEncoder, double encoderOffset, double ratio) {
        this.axon = axon;
        this.encoder = encoder;
        this.ratio = ratio;

        if (inversePower) {
            sign = -1;
        }
        this.inverseEncoder = inverseEncoder;

        this.encoderOffset = encoderOffset;
    }

    public void set(double power) {
        axon.set(power * sign);
    }

    public double get() {
        return axon.get() * sign;
    }

    /**
     * Reads the encoder and calculates angle
     */
    /**
     * Reads the encoder and calculates angle
     */
    public void readPos() {
        // Get raw angle
        double currentRawAngle = (encoder.getVoltage() / 3.3) * 360.0;

        if (inverseEncoder) {
            currentRawAngle = 360.0 - currentRawAngle;
        }

        // offset encoder at first read pos
        if (isFirstRead) {
            lastRawServoAngle = currentRawAngle;

            double startingAngle = currentRawAngle;

            // THE FIX: Catch the initial wraparound.
            // If the mechanism is resting slightly behind 0 on INIT, it will read 300+.
            // We subtract 360 so it starts at a proper negative number (e.g., -2 degrees) instead of 358.
            if (startingAngle > 300.0) {
                startingAngle -= 360.0;
            }

            continuousServoAngle = startingAngle + encoderOffset;
            isFirstRead = false;
            return;
        }


        double delta = currentRawAngle - lastRawServoAngle;

        // wrap around case
        if (delta > 180.0) {
            delta -= 360.0;
        } else if (delta < -180.0) {
            delta += 360.0;
        }

        continuousServoAngle += delta;

        lastRawServoAngle = currentRawAngle;
    }

    /**
     * @return The raw 0-360 angle from the encoder
     */
    public double getServoAngle() {
        return lastRawServoAngle;
    }

    /**
     * @return scaled for ratios
     */
    public double getScaledPos() {
        return (continuousServoAngle * ratio);
    }

    public double readAndGetVoltage() {
        return encoder.getVoltage();
    }

    public void resetEncoder(double offset) {
        this.encoderOffset = offset;
        this.isFirstRead = true;
    }
}
