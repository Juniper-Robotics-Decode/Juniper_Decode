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
    private ElapsedTime modeLockTimer = new ElapsedTime();

    public static double Sensor_Flicker = 0.3;
    public static boolean forceEmpty = false;

    private Telemetry telemetry;
    public status pocket1, pocket2, pocket3;
    public MotorWrapper spindexMotor;
    public int[] rVals = new int[3], gVals = new int[3], bVals = new int[3];

    public SpindexFSM(HWMapSpindex hwMap, Telemetry telemetry) {
        touchSensorMotorFSM = new TouchSensorMotorFSM(hwMap, telemetry);
        colorSensorsFSM = new ColorSensorFSM(hwMap, telemetry);
        pidChanges = new PIDChanges(hwMap, telemetry);
        this.telemetry = telemetry;
        this.spindexMotor = touchSensorMotorFSM.spindexMotor;
        this.mode = modes.INTAKNG;
        this.state = states.STOPPING_AT_TARGET;
    }

    public void updateState(double runtime, int r, int g, int b) {

        touchSensorMotorFSM.updateState();
        colorSensorsFSM.updateState();


        colorPocket(touchSensorMotorFSM.currentIndex, r, g, b);

//1
        if (forceEmpty) {
            pocket1 = status.EMPTY;
            pocket2 = status.EMPTY;
            pocket3 = status.EMPTY;
            mode = modes.INTAKNG;
        }


        boolean allFull = (pocket1 != status.EMPTY && pocket2 != status.EMPTY && pocket3 != status.EMPTY);
        boolean allEmpty = (pocket1 == status.EMPTY && pocket2 == status.EMPTY && pocket3 == status.EMPTY);


        if (!forceEmpty) {
            if (mode == modes.INTAKNG && allFull) {
                if (modeLockTimer.seconds() > Sensor_Flicker) {
                    mode = modes.SHOOTING;
                    modeLockTimer.reset();
                }
            }
            else if (mode == modes.SHOOTING && allEmpty) {
                if (modeLockTimer.seconds() > Sensor_Flicker) {
                    mode = modes.INTAKNG;
                    modeLockTimer.reset();
                    timer.reset();
                }
            }
            else {

                modeLockTimer.reset();
            }
        }


        switch (mode) {
            case INTAKNG:
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

                pidChanges.PIDMoveCalc(runtime);
                break;

            case SHOOTING:
                switch (state) {
                    case STOPPING_AT_TARGET: break;
                    case STOPPED_AT_TARGET: break;
                }
                spindexMotor.set(1.0);
                break;
        }

        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addData("--- SPINDEX FSM ---", "");
        telemetry.addData("Mode", mode);
        telemetry.addData("Motor Power", spindexMotor.get());
        telemetry.addData("Pocket 1", pocket1);
        telemetry.addData("Pocket 2", pocket2);
        telemetry.addData("Pocket 3", pocket3);
        telemetry.addData("Current Index", touchSensorMotorFSM.currentIndex);
        telemetry.addData("Target Angle", pidChanges.targetAngle);
        telemetry.addData("Force Empty", forceEmpty);
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