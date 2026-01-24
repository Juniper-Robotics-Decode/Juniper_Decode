package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

public class SpindexFSM {

    public enum states {
        STOPPING_AT_TARGET, // spindex moving to target slot
        STOPPED_AT_TARGET,   // spindex reached target slot
    }

    public enum modes {
        SHOOTING,
        INTAKNG,
    }

    private TouchSensorMotorFSM touchSensorMotorFSM;
    private ColorSensorFSM colorSensorsFSM;
    private states state;
    public  modes mode;
    private Telemetry telemetry; private String[] detectedMotif = new String[3];
    public int ball, noBall, x, y, currentIndex, target;
    public MotorWrapper spindexMotor;
    //1


    public SpindexFSM(HWMapSpindex hwMap, Telemetry telemetry) {
        touchSensorMotorFSM = new TouchSensorMotorFSM(hwMap, telemetry);
        colorSensorsFSM = new ColorSensorFSM(hwMap, telemetry);
        this.telemetry = telemetry;
        this.spindexMotor = touchSensorMotorFSM.spindexMotor;
        state = states.STOPPING_AT_TARGET;
        mode = modes.INTAKNG;

    }


    public void updateState(boolean shooting){ //Gamepad gamepad1,
        touchSensorMotorFSM.updateState();
        colorSensorsFSM.updateState();
        String currentMotif = colorSensorsFSM.getDetectedMotif();
        //angle calc
        double angle = Math.abs(spindexMotor.getAngle()) % 360;
        //assining pocket
        if (angle < 120) {
            detectedMotif[0] = currentMotif;
        } else if (angle < 240) {
            detectedMotif[1] = currentMotif;
        } else {
            detectedMotif[2] = currentMotif;
        }

        switch (state) {
            case STOPPING_AT_TARGET:
                state = states.STOPPING_AT_TARGET;
                telemetry.addData("State:", state);
                if (shooting){ //gamepad right trigger
                        spindexMotor.set(1);
                    mode = modes.SHOOTING;
                    spindexMotor.set(1);
                } else {
                    spindexMotor.set(0);
                }
                break;
            case STOPPED_AT_TARGET:
                state = states.STOPPED_AT_TARGET;
                break;
        }
        telemetry.addData("State:", state);
        telemetry.addData("Power:", spindexMotor.get());
        telemetry.addData("Slot 0", detectedMotif[0]);
        telemetry.addData("Slot 1", detectedMotif[1]);
        telemetry.addData("Slot 2", detectedMotif[2]);
        telemetry.update();
    }

    public void targetAngleCalculation() {
        touchSensorMotorFSM.spindexOffset(mode);
        double a = Math.abs(spindexMotor.getAngle());
        double relativePositionInRotation = a % 360;

        if (relativePositionInRotation < 120) {
            currentIndex = 1;
        } else if (relativePositionInRotation < 240 && relativePositionInRotation > 120) {
            currentIndex = 2;
        } else {
            currentIndex = 3;
        }

        for (int j = 0; j < detectedMotif.length; j++) {
            if (("Green".equals(detectedMotif[j])) || ("Purple".equals(detectedMotif[j]))) {
                ball = j;
            } else {
                noBall = j;
            }
        }

        x = currentIndex - ball;
        y = currentIndex - noBall;

        if (x > y) { // Counter-Clockwise/left
            target = (y * 120)+ touchSensorMotorFSM.offset;
            spindexMotor.setTarget(target);
        }

        if (y > x) { // Clockwise/Right
            target = (x * 120) + touchSensorMotorFSM.offset;
            spindexMotor.setTarget(target);
        }

        if (x == y) { // Counter-Clockwise/left
            target = (y * 120) + touchSensorMotorFSM.offset;
            spindexMotor.setTarget(target);
        }
    } // not needed anymore as shooting is different now
}
