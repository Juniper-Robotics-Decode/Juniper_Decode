package org.firstinspires.ftc.teamcode.Spindex;

import androidx.annotation.NonNull;

import com.bylazar.lights.Light;
import com.bylazar.lights.LightType;
import com.bylazar.lights.RGBIndicator;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class SpindexLightTest extends LinearOpMode{

    private enum rgbIndicators{
        INDICATOR1,
        INDICATOR2,
        INDICATOR3,
        NONE
    }
    private enum status {
        VIOLET,
        GREEN,
        WHITE
    }
    private TouchSensorMotorFSM touchSensorMotorFSM;
    private ColorSensorFSM colorSensorsFSM;
    private Telemetry telemetry;
    private RGBIndicator rgbIndicator1;
    private SpindexLightTest.rgbIndicators rgbIndicators;
    private status status;
    public status pocket1;
    public status pocket2;
    public status pocket3;
    public SpindexLightTest(HWMapSpindex hwMap, Telemetry telemetry) {
         touchSensorMotorFSM = new TouchSensorMotorFSM(hwMap, telemetry);
         colorSensorsFSM = new ColorSensorFSM(hwMap, telemetry, 1);
         rgbIndicator1 = hwMap.getRgbIndicator();
         this.telemetry = telemetry;
     }

     public void runOpMode() throws InterruptedException{
        waitForStart();
        while(opModeIsActive()){
            indicateTest();
            colorPocket(touchSensorMotorFSM.currentIndex);
            telemetry.addData("indicated","done");
            telemetry.update();
        }
    }

     public void indicateTest(){
        indicator();
        color();
        if (indicator() == rgbIndicators.INDICATOR1){
            if (color() == status.GREEN){
                rgbIndicator1.Companion.getGREEN();
                telemetry.addData("Indicator","1: GREEN");
            } else if (color() == status.VIOLET){
                rgbIndicator1.Companion.getVIOLET();
                telemetry.addData("Indicator","1: VIOLET");
            } else {
                rgbIndicator1.Companion.getWHITE();
                telemetry.addData("Indicator","1: WHITE");
            }
        } else if (indicator() == rgbIndicators.INDICATOR2) {
            if (color() == status.GREEN) {
//                rgbIndicator2.Companion.getGREEN();
                telemetry.addData("Indicator","2: GREEN");
            } else if (color() == status.VIOLET) {
//                rgbIndicator2.Companion.getVIOLET();
                telemetry.addData("Indicator","2: VIOLET");
            } else {
//                rgbIndicator2.Companion.getWHITE();
                telemetry.addData("Indicator","2: WHITE");
            }
        } else if (indicator() == rgbIndicators.INDICATOR3) {
            if (color() == status.GREEN) {
//                rgbIndicator3.Companion.getGREEN();
                telemetry.addData("Indicator","3: GREEN");
            } else if (color() == status.VIOLET) {
//                rgbIndicator3.Companion.getVIOLET();
                telemetry.addData("Indicator","3: VIOLET");
            } else {
//                rgbIndicator3.Companion.getWHITE();
                telemetry.addData("Indicator","3: WHITE");
            }
        } else {
            telemetry.addData("Indicator","NONE");
        }
        telemetry.update();
     }

    private rgbIndicators indicator() {
        if(touchSensorMotorFSM.atPosition1()){
            return rgbIndicators = rgbIndicators.INDICATOR1;
        } else if(touchSensorMotorFSM.atPosition2()){
            return rgbIndicators = rgbIndicators.INDICATOR2;
        } else if(touchSensorMotorFSM.atPosition3()){
            return rgbIndicators = rgbIndicators.INDICATOR3;
        } else{
            return rgbIndicators.NONE;
        }
    }
    private status color(){
    if(colorSensorsFSM.slot1IsGreen()){
        return status = status.GREEN;
    } else if(colorSensorsFSM.slot1IsPurple()){
        return status = status.VIOLET;
    } else{
        return status = status.WHITE;
        }
    }
    private void colorPocket(int pocket) {
        if(pocket == 1) {
            pocket1 = color();
            telemetry.addData("colorPocket",pocket1);
        } else if(pocket == 2) {
            pocket2 = color();
            telemetry.addData("colorPocket",pocket2);
        } else if(pocket == 3) {
            pocket3 = color();
            telemetry.addData("colorPocket",pocket3);
        }
        telemetry.addData("colorPocket",pocket);
    }

}
