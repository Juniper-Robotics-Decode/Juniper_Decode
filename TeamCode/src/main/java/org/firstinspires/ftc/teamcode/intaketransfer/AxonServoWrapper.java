package org.firstinspires.ftc.teamcode.intaketransfer;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

public class AxonServoWrapper {
    private final Servo axon;
    private final AnalogInput encoder;
    private double lastReadPosition;
    private double encoderOffset;
    private double inverseEncoderOffset;


    public AxonServoWrapper(Servo axon, AnalogInput encoder, boolean inversePower, boolean inverseEncoder, double encoderOffset) {
        this.axon = axon;
        this.encoder = encoder;
        if (inversePower) {
            double sign = -1;
        }
        if (inverseEncoder) {
            inverseEncoderOffset = 1;
        }

        this.encoderOffset = encoderOffset;
    }

    /*public void set(double power) {
        axon.set(power * sign);
    }
*/
  /*  public double get() {
        return axon.get() * sign;
    }
*/


    /**
     * @param pos 0-1
     */
    public void setPos(double pos) {
        axon.setPosition(pos - encoderOffset);
    }

    public double readPos() {

        lastReadPosition = (encoder.getVoltage() / 3.3 - encoderOffset);

        return lastReadPosition;
    }

    public double getVoltage() {
        return encoder.getVoltage();
    }

    public double getLastReadPos() {
        return lastReadPosition;
    }

    /**
     * @param offset 0-1
     */
    public void setEncoderOffset(double offset) {
        this.encoderOffset = offset;

    }
}
