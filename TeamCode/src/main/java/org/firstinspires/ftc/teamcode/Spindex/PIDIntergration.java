package org.firstinspires.ftc.teamcode.Spindex;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

@TeleOp
public class PIDIntergration extends LinearOpMode{
    private ColorSensorFSM colorSensorsFSM;
    private TouchSensorMotorFSM touchSensorMotorFSM;
    private SpindexFSM spindexFSM;
    private PIDChanges pidChanges;
    private status status;
    public status pocket1;
    public status pocket2;
    public status pocket3;
    public double targetAngle;
    private MotorWrapper spindexMotor;
    public static boolean shooting = false;
    public int[] rVals = new int[3];
    public int[] gVals = new int[3];
    public int[] bVals = new int[3];
    private ElapsedTime timer = new ElapsedTime();
    //copy paste all this code to juniper decode of Juniper decode and not color sensor code
    private enum status {
        PURPLE,
        GREEN,
        EMPTY
    }
    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HWMapSpindex hwMap = new HWMapSpindex(hardwareMap);

        colorSensorsFSM = new ColorSensorFSM(hwMap,telemetry,1);
        touchSensorMotorFSM = new TouchSensorMotorFSM(hwMap, telemetry);
        spindexFSM = new SpindexFSM(hwMap, telemetry);

        spindexMotor = new MotorWrapper(hwMap.getSpindexMotor(),false,1, 537.7);


        pidChanges = new PIDChanges(hwMap,telemetry);

        waitForStart();
        while(opModeIsActive()){
            colorSensorsFSM.updateState();
            touchSensorMotorFSM.updateState();
            spindexFSM.updateState(shooting);

            int r = hwMap.getColorSensor1().red();
            int g = hwMap.getColorSensor1().green();
            int b = hwMap.getColorSensor1().blue();

            colorPocket(touchSensorMotorFSM.currentIndex, r, g, b);
            if (timer.seconds() > 2.5) {
                if (pocket1 == status.EMPTY) {
                    pidChanges.targetAngle = 360;
                    timer.reset();
                } else if (pocket2 == status.EMPTY) {
                    pidChanges.targetAngle = 120 ;
                    timer.reset();
                } else if (pocket3 == status.EMPTY) {
                    pidChanges.targetAngle = 240;
                    // 60 is offset, pretty sure thats not issue, this is just for simplicitytouchSensorMotorFSM.offset;
                    timer.reset();
                } else {
                    spindexMotor.set(1);
                }
            }

            pidChanges.PIDMoveCalc(getRuntime());
            telemetry.addData("Pocket 1 [0]", pocket1);
            telemetry.addData("P1 RGB", rVals[0] + ", " + gVals[0] + ", " + bVals[0]);
            telemetry.addData("Pocket 2 [1]", pocket2);
            telemetry.addData("P2 RGB", rVals[1] + ", " + gVals[1] + ", " + bVals[1]);
            telemetry.addData("Pocket 3 [2]", pocket3);
            telemetry.addData("P3 RGB", rVals[2] + ", " + gVals[2] + ", " + bVals[2]);

            telemetry.addData("Current Index", touchSensorMotorFSM.currentIndex);
            telemetry.addData("Target Angle", pidChanges.targetAngle);
            telemetry.addData("Current Angle",pidChanges.currentPosition);
            telemetry.update();

        }
    }

    private status color(int pocket){
        if(spindexFSM.slotIsGreen(pocket)){
            return status = status.GREEN;
        } else if(spindexFSM.slotIsPurple(pocket)){
            return status = status.PURPLE;
        } else{
            return status = status.EMPTY;
        }
    }

    private void colorPocket(int pocket, int r, int g, int b) {
        if(pocket == 1) {
            pocket1 = color(0);
            rVals[0] = r; gVals[0] = g; bVals[0] = b;
        } else if(pocket == 2) {
            pocket2 = color(1);
            rVals[1] = r; gVals[1] = g; bVals[1] = b;
        } else if(pocket == 3) {
            pocket3 = color(2);
            rVals[2] = r; gVals[2] = g; bVals[2] = b;
        }
    }
}