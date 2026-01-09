package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;
@TeleOp
public class ColorSensorFSM{
    public enum states{
        EMPTY,
        GREEN,
        PURPLE,
    }

    public RevColorSensorV3 CS;
    public states state;
    public  MotorWrapper spindexMotor;
    private Telemetry telemetry;
    private String detectedMotif;
    private boolean[] full = new boolean[3];
    private boolean[] green = new boolean[3];
    private boolean[] purple = new boolean[3];
    private final String greenStr = "Green";
    private final String purpleStr = "Purple";
    private final String emptyStr = "Empty";

    public ColorSensorFSM(HWMapSpindex hwMap, Telemetry telemetry, int sensorID) {

        if (sensorID == 1) CS = hwMap.getColorSensor1();
        else if (sensorID == 2) CS = hwMap.getColorSensor2();
        else CS = hwMap.getColorSensor3();

        this.state = states.EMPTY;
        this.detectedMotif = emptyStr;


        this.telemetry = telemetry;

        state= states.EMPTY; // default
    }
    public void updateState() {
        // FIX: Ensure sensor exists before reading
        if (CS == null) return;

        detectedMotif = colorDetector(CS);

        if (detectedMotif.equals(greenStr)) {
            state = states.GREEN;
        } else if (detectedMotif.equals(purpleStr)) {
            state = states.PURPLE;
        } else {
            state = states.EMPTY;
        }





    }
    public String colorDetector (RevColorSensorV3 cs){
        int blue = cs.blue();
        int green = cs.green();
        int red = cs.red();
        double dREAD = cs.getDistance(DistanceUnit.MM);

        if (dREAD <= 60) {
            if (green > red && green > blue) {
                return greenStr;

            } else if (blue > red && blue > green) {
                return purpleStr;
            }
        } else {
            return emptyStr;
        }
        return "";
    }
    public String getDetectedMotif() {
        return detectedMotif;
    }
    public boolean slot1IsEmpty() {
        return state == states.EMPTY;
    }

    public boolean slot1IsGreen() {
        return state == states.GREEN;
    }

    public boolean slot1IsPurple() {
        return state == states.PURPLE;
    }

}