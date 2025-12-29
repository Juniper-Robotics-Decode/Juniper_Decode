
package org.firstinspires.ftc.teamcode.core;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class motorWrapperSpindex {
    private final MotorEx motorEx;


    public motorWrapperSpindex(MotorEx motorEx) {
        this.motorEx = motorEx;
    }

    // POWER

    /**
     * Description: The set method is a wrapper of the motor set method
     *
     * @param : the power to set the motor at
     */

    public void set(double power) {
        motorEx.set(power);
    }

    public double get() {
        return motorEx.get();
    }
}