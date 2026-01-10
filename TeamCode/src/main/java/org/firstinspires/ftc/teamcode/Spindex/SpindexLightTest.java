package org.firstinspires.ftc.teamcode.Spindex;

import com.bylazar.lights.RGBIndicator;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class SpindexLightTest extends LinearOpMode{

    private enum status {
        PURPLE,
        GREEN,
        WHITE,
        ERROR
    }
    private HWMapSpindex hwMap;
    private TouchSensorMotorFSM touchSensorMotorFSM;
    private ColorSensorFSM colorSensorsFSM;
    private Telemetry telemetry;
    private RGBIndicator RI1 = hwMap.getRgbIndicator1();
    private RGBIndicator RI2 = hwMap.getRgbIndicator2();
    private RGBIndicator RI3 = hwMap.getRgbIndicator3();
    private status status;
    public status pocket1;
    public status pocket2;
    public status pocket3;


    public SpindexLightTest() {
     }

     public void runOpMode() throws InterruptedException{
        waitForStart();
        int l = 0;
        while(opModeIsActive() && !isStopRequested()){
            if(l < 200) {
                l += 1;
                indicateTest();
                colorPocket(touchSensorMotorFSM.currentIndex);
                telemetry.addData("indicated", "done");
                telemetry.addData("l",l);
                telemetry.update();
            }
        }
    }

     public void indicateTest() {
         int ind = indicator();
         status col = color(ind);
         switch (ind) {
             case 1:
                 setIndicator(RI1, col);
                 telemetry.addData("indicateTest", "RI1" + col);
                 break;
             case 2:
                 setIndicator(RI2, col);
                 telemetry.addData("indicateTest", "RI2" + col);
                 break;
             case 3:
                 setIndicator(RI3, col);
                 telemetry.addData("indicateTest", "RI3" + col);
                 break;
             default:
                 telemetry.addData("indicateTest", "none");
         }
     }

     public void setIndicator(RGBIndicator RI, status color) {
         if (color == status.GREEN) {
             RI.Companion.getGREEN();
             telemetry.addData("companion","got green");
         } else if (color == status.PURPLE) {
             RI.Companion.getVIOLET();
             telemetry.addData("companion","got purple");
         } else if (color == status.WHITE) {
             RI.Companion.getWHITE();
             telemetry.addData("companion","got white");
         } else{
             RI.Companion.getAZURE();
             telemetry.addData("companion","got bwoo");
         }
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
                return status.GREEN;
            } else if (colorSensorsFSM.slotIsPurple(sensor)) {
                return status.PURPLE;
            } else if(colorSensorsFSM.slotIsEmpty(sensor)){
                return status.WHITE;
            } else{
                return status.ERROR;
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
