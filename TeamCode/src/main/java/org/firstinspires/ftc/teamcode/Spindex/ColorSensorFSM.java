package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;
@TeleOp
public class ColorSensorFSM{
    public enum states{
        EMPTY,
        GREEN,
        PURPLE,
    }

    public static RevColorSensorV3 CS;
    public static states state;
    public  MotorWrapper spindexMotor;
    private Telemetry telemetry;
    private String detectedMotif;
    private boolean[] full = new boolean[3];
    private boolean[] green = new boolean[3];
    private boolean[] purple = new boolean[3];
    private final String greenStr = "Green";
    private final String purpleStr = "Purple";
    private final String emptyStr = "Empty";
    private double d;
    public int ball, noBall, x, y, currentIndex, target;

    public ColorSensorFSM(HWMap hwMap, Telemetry telemetry) {

        CS = hwMap.getColorSensor1();
;       this.CS = CS;
        this.state = states.EMPTY;
        this.detectedMotif = emptyStr;


        this.telemetry = telemetry;

        state= states.EMPTY; // default
    }
    public void updateState() {

        detectedMotif = colorDetector(CS);

        if (detectedMotif.equals(greenStr)) {
            state = states.GREEN;
        } else if (detectedMotif.equals(purpleStr)) {
            state = states.PURPLE;
        } else {
            state = states.EMPTY;
        }

        d = CS.getDistance(DistanceUnit.MM);
 



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
