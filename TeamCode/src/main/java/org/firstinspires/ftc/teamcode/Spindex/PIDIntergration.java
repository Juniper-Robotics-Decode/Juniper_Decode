package org.firstinspires.ftc.teamcode.Spindex;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

@TeleOp
public class PIDIntergration extends LinearOpMode {
    private ColorSensorFSM colorSensorsFSM;
    private TouchSensorMotorFSM touchSensorMotorFSM;
    private SpindexFSM spindexFSM;
    private PIDChanges pidChanges;

    // Using the status enum from SpindexFSM to avoid conflicts
    public SpindexFSM.status pocket1;
    public SpindexFSM.status pocket2;
    public SpindexFSM.status pocket3;

    private MotorWrapper spindexMotor;
    public boolean shooting = false;
    public int[] rVals = new int[3];
    public int[] gVals = new int[3];
    public int[] bVals = new int[3];
    private ElapsedTime timer = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HWMapSpindex hwMap = new HWMapSpindex(hardwareMap);

        colorSensorsFSM = new ColorSensorFSM(hwMap, telemetry, 1);
        touchSensorMotorFSM = new TouchSensorMotorFSM(hwMap, telemetry);
        spindexFSM = new SpindexFSM(hwMap, telemetry);
        pidChanges = new PIDChanges(hwMap, telemetry);
        spindexMotor = new MotorWrapper(hwMap.getSpindexMotor(), false, 1, 537.7);

        waitForStart();
        while (opModeIsActive()) {
            int r = hwMap.getColorSensor1().red();
            int g = hwMap.getColorSensor1().green();
            int b = hwMap.getColorSensor1().blue();

            // FIXED: Added missing parameters
            spindexFSM.updateState(shooting, getRuntime(), r, g, b);

            colorPocket(touchSensorMotorFSM.currentIndex, r, g, b);

            if (timer.seconds() > 2.5) {
                if (pocket1 == SpindexFSM.status.EMPTY) {
                    pidChanges.targetAngle = 360;
                    timer.reset();
                } else if (pocket2 == SpindexFSM.status.EMPTY) {
                    pidChanges.targetAngle = 120;
                    timer.reset();
                } else if (pocket3 == SpindexFSM.status.EMPTY) {
                    pidChanges.targetAngle = 240;
                    timer.reset();
                } else {
                    spindexMotor.set(1);
                }
            }

            pidChanges.PIDMoveCalc(getRuntime());

            telemetry.addData("Pocket 1 [0]", pocket1);
            telemetry.addData("Pocket 2 [1]", pocket2);
            telemetry.addData("Pocket 3 [2]", pocket3);
            telemetry.addData("Current Angle", pidChanges.currentPosition);
            telemetry.update();
        }
    }

    private void colorPocket(int pocket, int r, int g, int b) {
        if (pocket == 1) {
            pocket1 = spindexFSM.pocket1;
            rVals[0] = r; gVals[0] = g; bVals[0] = b;
        } else if (pocket == 2) {
            pocket2 = spindexFSM.pocket2;
            rVals[1] = r; gVals[1] = g; bVals[1] = b;
        } else if (pocket == 3) {
            pocket3 = spindexFSM.pocket3;
            rVals[2] = r; gVals[2] = g; bVals[2] = b;
        }
    }
}