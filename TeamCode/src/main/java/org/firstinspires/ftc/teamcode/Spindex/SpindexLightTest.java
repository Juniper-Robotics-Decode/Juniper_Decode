package org.firstinspires.ftc.teamcode.Spindex;

import com.bylazar.lights.RGBIndicator;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import com.arcrobotics.ftclib.hardware.ServoEx;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Light;

import com.qualcomm.robotcore.hardware.PwmControl;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class SpindexLightTest extends LinearOpMode{

    public enum status {
        PURPLE,
        GREEN,
        WHITE,
        ERROR
    }
    private HWMapSpindex hwMap;
    private TouchSensorMotorFSM touchSensorMotorFSM;
    private ColorSensorFSM colorSensorsFSM;
    private Telemetry telemetry;
    private ServoEx RI1;
    private ServoEx RI2;
    private ServoEx RI3;
    public status pocket1;
    public status pocket2;
    public status pocket3;


    public SpindexLightTest() {
        hwMap = new HWMapSpindex(hardwareMap);
        touchSensorMotorFSM = new TouchSensorMotorFSM(hwMap, telemetry);
        colorSensorsFSM = new ColorSensorFSM(hwMap, telemetry,1);
        this.telemetry = telemetry;
        RI1 = hwMap.getRgbIndicator1();
        RI2 = hwMap.getRgbIndicator2();
        RI3 = hwMap.getRgbIndicator3();
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

     public void setIndicator(ServoEx RI, status color) {
         if (color == status.GREEN) {
             RI.setPosition(0.5);
             telemetry.addData("companion","got green");
         } else if (color == status.PURPLE) {
             RI.setPosition(0.7);
             telemetry.addData("companion","got purple");
         } else if (color == status.WHITE) {
             RI.setPosition(1);
             telemetry.addData("companion","got white");
         } else{
             RI.setPosition(0.6);
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
