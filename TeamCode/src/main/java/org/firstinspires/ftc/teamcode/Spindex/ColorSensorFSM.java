package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

public class ColorSensorFSM{
    public enum states{
        EMPTY,
        GREEN,
        PURPLE,
        DUST_CLOGGED,
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
    public RevColorSensorV3 CS1;
    public RevColorSensorV3 CS2;
    public int activeSensorID = 1;
    public ColorSensorFSM(HWMapSpindex hwMap, Telemetry telemetry, int activeSensorID) {
        CS1 = hwMap.getColorSensor1();
        CS2 = hwMap.getColorSensor2();
        if (activeSensorID == 1) CS = CS1;
        else if (activeSensorID == 2) CS = CS2;

        this.state = states.EMPTY;
        this.detectedMotif = emptyStr;


        this.telemetry = telemetry;

        state= states.EMPTY; // default
    }
    public void updateState() {

        if (activeSensorID == 1) {
            CS = CS1;
        } else if (activeSensorID == 2) {
            CS = CS2;
        }

        if (CS == null) return;
        detectedMotif = colorDetector(CS);
        // Fail-Switch
        if (detectedMotif.equals("CLOGGED")) {
            if (activeSensorID == 1) {
                activeSensorID = 2; // switch to backup sensor!
            }
            else {
                activeSensorID = 1;
            }

            state = states.DUST_CLOGGED;
            telemetry.addData("SENSOR ALERT", "Clogged! Switched to Sensor " + activeSensorID);
        }
        else if (detectedMotif.equals(greenStr)) {
            state = states.GREEN;
        } else if (detectedMotif.equals(purpleStr)) {
            state = states.PURPLE;
        } else {
            state = states.EMPTY;
        }
    }

    public String colorDetector(RevColorSensorV3 cs) {
        if (cs == null) return emptyStr;

        int blue = cs.blue();
        int green = cs.green();
        int red = cs.red();
        double dREAD = cs.getDistance(DistanceUnit.MM);

        // get the average color
        double mean = (red + green + blue) / 3.0;
        // find how far colors are from average and square them
        double variance = (Math.pow(red - mean, 2) + Math.pow(green - mean, 2) + Math.pow(blue - mean, 2)) / 3.0;
        // square root to get value, and square root cancels out the squaring above -------------------^
        double stdDev = Math.sqrt(variance);

        if (stdDev < 10) {
            return "CLOGGED";
        }

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
    public boolean slot1IsEmpty() {
        colorDetector(CS1);
        telemetry.addData("CS1",String.valueOf(CS1));
        return state == states.EMPTY;
    }



    public String getDetectedMotif() {
        return detectedMotif;
    }
}