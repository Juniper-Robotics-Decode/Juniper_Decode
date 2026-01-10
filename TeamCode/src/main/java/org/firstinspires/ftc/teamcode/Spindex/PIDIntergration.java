package org.firstinspires.ftc.teamcode.Spindex;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.ftdi.VendorAndProductIds;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;
import org.firstinspires.ftc.teamcode.Spindex.PIDChanges;

public class PIDIntergration extends LinearOpMode{
    private ColorSensorFSM colorSensorsFSM;
    private TouchSensorMotorFSM touchSensorMotorFSM;
    private PIDChanges pidChanges;
    private status status;
    public status pocket1;
    public status pocket2;
    public status pocket3;
    public double targetAngle;
    private MotorWrapper spindexMotor;
    private enum status {
        PURPLE,
        GREEN,
        EMPTY
    }
    public PIDIntergration(HWMapSpindex hwMap, Telemetry telemetry){
            colorSensorsFSM = new ColorSensorFSM(hwMap,telemetry,1);
            touchSensorMotorFSM = new TouchSensorMotorFSM(hwMap, telemetry);
            spindexMotor = new MotorWrapper(hwMap.getSpindexMotor(),false,1, 537.7);

        pidChanges = new PIDChanges(hwMap,telemetry);

        this.telemetry = telemetry;
    }
    public void runOpMode() throws InterruptedException{
        waitForStart();
        while(opModeIsActive()){
            colorPocket(touchSensorMotorFSM.currentIndex);
            pidChanges.PIDMoveCalc(getRuntime());
            if(pocket1 == status.EMPTY) {
                //move to it
            } else if (pocket2 == status.EMPTY) {
                //move to it
                } else if (pocket3 == status.EMPTY) {
                //move to it
            } else{
                spindexMotor.set(1);

            }            //move to pocket
            //intake ball and assign that pocket the ball color
            //go till full and then spin unlimited

        }
    }
    //pocket * 120 = targetangle
    private status color(){
        if(colorSensorsFSM.slot1IsGreen()){
            return status = status.GREEN;
        } else if(colorSensorsFSM.slot1IsPurple()){
            return status = status.PURPLE;
        } else{
            return status = status.EMPTY;
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
