package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;
@TeleOp
public class ColorSensorsFSM {
    public enum states{
        EMPTY,
        GREEN,
        PURPLE,
    }

    public static RevColorSensorV3 CS1;
    public static RevColorSensorV3 CS2;
    public static RevColorSensorV3 CS3;
    public static states state1;
    public static states state2;
    public static states state3;
    private Telemetry telemetry;
    private String[] detectedMotif = new String[3];//color detection usage
    private boolean[] full = new boolean[3];
    private boolean[] green = new boolean[3];
    private boolean[] purple = new boolean[3];
    private final String greenStr = "Green";
    private final String purpleStr = "Purple";
    private final String emptyStr = "Empty";
    private double d1;
    private double d2;
    private double d3;

    public ColorSensorsFSM(HWMap hwMap, Telemetry telemetry) {

        CS1 = hwMap.getColorSensor1();
        CS2 = hwMap.getColorSensor2();
        CS3 = hwMap.getColorSensor3();

        this.telemetry = telemetry;

        state1= states.EMPTY; // default
        state2= states.EMPTY; // default
        state3= states.EMPTY; // default
    }
    public void updateState() {

        if (detectedMotif[0].equals(greenStr)) {
            state1= states.GREEN;
        } else if (detectedMotif[0].equals(purpleStr)) {
            state1= states.PURPLE;
        } else {
            state1= states.EMPTY;
        }

        if (detectedMotif[1].equals(greenStr)) {
            state2= states.GREEN;
        } else if (detectedMotif[1].equals(purpleStr)) {
            state2= states.PURPLE;
        } else {
            state2= states.EMPTY;
        }

        if (detectedMotif[2].equals(greenStr)) {
            state3= states.GREEN;
        } else if (detectedMotif[2].equals(purpleStr)) {
            state3= states.PURPLE;
        } else {
            state3= states.EMPTY;
        }

        detectedMotif[0] = colorDetector(CS1);
        detectedMotif[1] = colorDetector(CS2);
        detectedMotif[2] = colorDetector(CS3);


        d1 = CS1.getDistance(DistanceUnit.MM);
        d2 = CS2.getDistance(DistanceUnit.MM);
        d3 = CS3.getDistance(DistanceUnit.MM);



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

    public boolean slot1IsEmpty() {
        return state1 == states.EMPTY;
    }

    public boolean slot1IsGreen() {
        return state1 == states.GREEN;
    }

    public boolean slot1IsPurple() {
        return state1 == states.PURPLE;
    }

    public boolean slot2IsEmpty() {
        return state2 == states.EMPTY;
    }

    public boolean slot2IsGreen() {
        return state2 == states.GREEN;
    }

    public boolean slot2IsPurple() {
        return state2 == states.PURPLE;
    }

    public boolean slot3IsEmpty() {
        return state3 == states.EMPTY;
    }

    public boolean slot3IsGreen() {
        return state3 == states.GREEN;
    }

    public boolean slot3IsPurple() {
        return state3 == states.PURPLE;
    }

}
