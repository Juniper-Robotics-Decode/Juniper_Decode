package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

@TeleOp
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
    private ColorSensorFSM cs1;
    private ColorSensorFSM cs2;
    private ColorSensorFSM cs3;
    private Telemetry telemetry; private String[] detectedMotif = new String[3];
    public int ball, noBall, x, y, currentIndex, target;
    public MotorWrapper spindexMotor;

    public SpindexFSM(HWMapSpindex hwMap, Telemetry telemetry) {
        touchSensorMotorFSM = new TouchSensorMotorFSM(hwMap, telemetry);
        colorSensorsFSM = new ColorSensorFSM(hwMap, telemetry);
        this.telemetry = telemetry;
        this.cs1 = cs1;
        this.cs2 = cs2;
        this.cs3 = cs3;
        state = states.STOPPING_AT_TARGET;
        mode = modes.INTAKNG;

    }


     public void updateState(boolean shooting){ //Gamepad gamepad1,
         targetAngleCalculation();
         touchSensorMotorFSM.updateState();
         colorSensorsFSM.updateState();
         cs1.updateState();
         cs2.updateState();
         cs3.updateState();

         detectedMotif[0] = cs1.getDetectedMotif();
         detectedMotif[1] = cs2.getDetectedMotif();
         detectedMotif[2] = cs3.getDetectedMotif();

         switch (state) {
             case STOPPING_AT_TARGET:
                 state = states.STOPPING_AT_TARGET;
                 telemetry.addData("State:", state);
                 if (shooting){//boolean replace:gamepad1.square || gamepad1.circle || gamepad1.triangle
                     //   targetAngleCalculation(hwMap);
                     //   touchSensorMotorFSM.spindexOffset(mode);
                     //   spindexMotor.setTarget(target);
                     mode = modes.SHOOTING;
                     spindexMotor.set(0.5);
                     //spindex motor run to position command thing
                 }
             case STOPPED_AT_TARGET:
                 state = states.STOPPED_AT_TARGET;
                 telemetry.addData("State:", state);
                 break;
         }
     }
    //target angle calculation methods and stuff will be added into the switch statement
    //the offset will also be there like after finding target, add offset, then move
// need to add the actual motor movement code here with the offset and stuff

    //telemetry
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
            if ((detectedMotif[j].equals("Green")) || (detectedMotif[j].equals("Purple"))) {
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
    }

}
