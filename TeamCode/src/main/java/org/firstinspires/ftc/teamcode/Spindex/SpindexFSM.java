package org.firstinspires.ftc.teamcode.Spindex;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;
@Config
public class SpindexFSM {
    public enum modes { INTAKNG, SHOOTING }
    public enum states { STOPPING_AT_TARGET, STOPPED_AT_TARGET }
    public enum status { PURPLE, GREEN, EMPTY }

    public modes mode;
    public states state;
    private TouchSensorMotorFSM touchSensorMotorFSM;
    private ColorSensorFSM colorSensorsFSM;
    private PIDChanges pidChanges;
    private ElapsedTime timer = new ElapsedTime();
    private Telemetry telemetry;

    public status pocket1, pocket2, pocket3;
    public MotorWrapper spindexMotor;
    public int[] rVals = new int[3], gVals = new int[3], bVals = new int[3];

    public static boolean forceEmpty; //to simualte shooting to succesfully and properly test

    public SpindexFSM(HWMapSpindex hwMap, Telemetry telemetry) {
        this.spindexMotor = new MotorWrapper(hwMap.getSpindexMotor(), false, 1, 537.7);
        touchSensorMotorFSM = new TouchSensorMotorFSM(hwMap, telemetry, this.spindexMotor);
        colorSensorsFSM = new ColorSensorFSM(hwMap, telemetry);
        pidChanges = new PIDChanges(hwMap, telemetry, this.spindexMotor);
        this.telemetry = telemetry;
        this.mode = modes.INTAKNG;
        this.state = states.STOPPING_AT_TARGET;
    }
    // private ElapsedTime modeLockTimer = new ElapsedTime();
   // private final double FLICKER_TIME = 0.3; // 300 milliseconds to prevent code being wonky because of sensor flicking
    public void updateState(double runtime, int r, int g, int b) {
        spindexMotor.readPosition();
        touchSensorMotorFSM.updateState();
        colorSensorsFSM.updateState();
        pidChanges.PIDMoveCalc(runtime);
        colorPocket(touchSensorMotorFSM.currentIndex, r, g, b);
        if (forceEmpty) {
            pocket1 = status.EMPTY;
            pocket2 = status.EMPTY;
            pocket3 = status.EMPTY;
        }
        boolean allFull = (pocket1 != status.EMPTY && pocket2 != status.EMPTY && pocket3 != status.EMPTY);
        boolean allEmpty = (pocket1 == status.EMPTY && pocket2 == status.EMPTY && pocket3 == status.EMPTY);
        if (mode == modes.INTAKNG) {
            if (allFull) {
                    mode = modes.SHOOTING;

            } else {
                    mode = modes.INTAKNG;
                  //timer.reset(); //reset the 2.5
            }
        }
        else if (mode == modes.SHOOTING) {
            if (allEmpty) {
                    mode = modes.INTAKNG;
                    timer.reset(); //reset the 2.5
            } else {
                    mode = modes.SHOOTING;
            }
        }

        switch (mode) {
            case INTAKNG:
                //check if all full
                if (pocket1 != status.EMPTY && pocket2 != status.EMPTY && pocket3 != status.EMPTY) {
                    mode = modes.SHOOTING;
                }
            //if not all full then need to intake and stuff
                if (timer.seconds() > 2.5) {
                    if (pocket1 == status.EMPTY) {
                        pidChanges.targetAngle = 360;
                        timer.reset();
                    }
                    else if (pocket2 == status.EMPTY) {
                        pidChanges.targetAngle = 120;
                        timer.reset();
                    }
                    else if (pocket3 == status.EMPTY) {
                        pidChanges.targetAngle = 240;
                        timer.reset();
                    }
                }
                //FOR NOW PID only moving when intaking
                pidChanges.PIDMoveCalc(runtime);
                break;

            case SHOOTING:
                switch (state) {
                    case STOPPING_AT_TARGET:
                        // later
                        break;
                    case STOPPED_AT_TARGET:
                        // later
                        break;
                }

                // CONTINUOUS SPIN until shooting button pressed
                spindexMotor.set(1.0);
                break;
        }

        // Telemetry exactly from original integration code
        telemetry.addData("State:", state);
        telemetry.addData("Power:", spindexMotor.get());
        telemetry.addData("Pocket 1 [0]", pocket1);
        telemetry.addData("P1 RGB", rVals[0] + ", " + gVals[0] + ", " + bVals[0]);
        telemetry.addData("Pocket 2 [1]", pocket2);
        telemetry.addData("P2 RGB", rVals[1] + ", " + gVals[1] + ", " + bVals[1]);
        telemetry.addData("Pocket 3 [2]", pocket3);
        telemetry.addData("P3 RGB", rVals[2] + ", " + gVals[2] + ", " + bVals[2]);
        telemetry.addData("Current Index", touchSensorMotorFSM.currentIndex);
        telemetry.addData("Target Angle", pidChanges.targetAngle);
        telemetry.addData("Current Angle", pidChanges.currentPosition);
    }

    private status color(int pocketIndex) {
        String motif = colorSensorsFSM.getDetectedMotif();
        if (motif.equals("Green")) return status.GREEN;
        else if (motif.equals("Purple")) return status.PURPLE;
        else return status.EMPTY;
    }

    private void colorPocket(int pocket, int r, int g, int b) {
        if(pocket == 1) { pocket1 = color(0); rVals[0] = r; gVals[0] = g; bVals[0] = b; }
        else if(pocket == 2) { pocket2 = color(1); rVals[1] = r; gVals[1] = g; bVals[1] = b; }
        else if(pocket == 3) { pocket3 = color(2); rVals[2] = r; gVals[2] = g; bVals[2] = b; }
    }
}
