package org.firstinspires.ftc.teamcode.Spindex;

import com.bylazar.lights.RGBIndicator;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class SpindexLightTest extends LinearOpMode{

    private enum status {
        VIOLET,
        GREEN,
        WHITE,
        ERROR
    }
    private TouchSensorMotorFSM touchSensorMotorFSM;
    private ColorSensorFSM colorSensorsFSM;
    private Telemetry telemetry;
    private RGBIndicator RI1;
    private RGBIndicator RI2;
    private RGBIndicator RI3;
    private status status;
    public status pocket1;
    public status pocket2;
    public status pocket3;
    public SpindexLightTest(HWMapSpindex hwMap, Telemetry telemetry) {
         touchSensorMotorFSM = new TouchSensorMotorFSM(hwMap, telemetry);
         colorSensorsFSM = new ColorSensorFSM(hwMap, telemetry, 1);
        RI1 = hwMap.getRgbIndicator1();
        RI2 = hwMap.getRgbIndicator2();
        RI3 = hwMap.getRgbIndicator3();
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
        if (indicator() == 1){
            if (color(1) == status.GREEN){
                RI1.Companion.getGREEN();
                telemetry.addData("Indicator","1: GREEN");
            } else if (color(1) == status.VIOLET){
                RI1.Companion.getVIOLET();
                telemetry.addData("Indicator","1: VIOLET");
            } else {
                RI1.Companion.getWHITE();
                telemetry.addData("Indicator","1: WHITE");
            }
        } else if (indicator() == 3) {
            if (color(2) == status.GREEN) {
                RI2.Companion.getGREEN();
                telemetry.addData("Indicator","2: GREEN");
            } else if (color(2) == status.VIOLET) {
                RI2.Companion.getVIOLET();
                telemetry.addData("Indicator","2: VIOLET");
            } else {
                RI2.Companion.getWHITE();
                telemetry.addData("Indicator","2: WHITE");
            }
        } else if (indicator() == 3) {
            if (color(3) == status.GREEN) {
                RI3.Companion.getGREEN();
                telemetry.addData("Indicator","3: GREEN");
            } else if (color(3) == status.VIOLET) {
                RI3.Companion.getVIOLET();
                telemetry.addData("Indicator","3: VIOLET");
            } else {
                RI3.Companion.getWHITE();
                telemetry.addData("Indicator","3: WHITE");
            }
        } else {
            telemetry.addData("Indicator","NONE");
        }
        telemetry.update();
     }

    private int indicator() {
        if(touchSensorMotorFSM.atPosition1()){
            return 1;
        } else if(touchSensorMotorFSM.atPosition2()){
            return 2;
        } else if(touchSensorMotorFSM.atPosition3()){
            return 3;
        } else{
            return 0;
        }
    }
    private status color(int sensor) {
            if (colorSensorsFSM.slotIsGreen(sensor)) {
                return status = status.GREEN;
            } else if (colorSensorsFSM.slotIsPurple(sensor)) {
                return status = status.VIOLET;
            } else if(colorSensorsFSM.slotIsEmpty(sensor)){
                return status = status.WHITE;
            } else{
                return status = status.ERROR;
            }
    }
    private void colorPocket(int pocket) {
        if(pocket == 1) {
            pocket1 = color(pocket);
            telemetry.addData("colorPocket",pocket1);
        } else if(pocket == 2) {
            pocket2 = color(pocket);
            telemetry.addData("colorPocket",pocket2);
        } else if(pocket == 3) {
            pocket3 = color(pocket);
            telemetry.addData("colorPocket",pocket3);
        }
        telemetry.addData("colorPocket",pocket);
    }

}
