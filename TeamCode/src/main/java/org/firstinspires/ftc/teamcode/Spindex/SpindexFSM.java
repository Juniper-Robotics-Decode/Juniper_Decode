package org.firstinspires.ftc.teamcode.Spindex;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
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
    private spindexMotorFSM spindexMotorFSM;
    private ColorSensorFSM colorSensorsFSM;
    private IntakeMotorFSM intakeMotorFSM;
    private PIDChanges pidChanges;
    private ElapsedTime timer = new ElapsedTime();
    private Telemetry telemetry;

    public status pocket1, pocket2, pocket3;
    public MotorWrapper spindexMotor;
    public int[] rVals = new int[3], gVals = new int[3], bVals = new int[3];

    public boolean forceEmpty; //to simualte shooting to succesfully and properly test

    public boolean lastTriangle, intakeEnabled, lastSquare = false, manualEject = false;
    public SpindexFSM(HWMapSpindex hwMap, Telemetry telemetry) {
        this.spindexMotor = new MotorWrapper(hwMap.getSpindexMotor(), false, 1, 537.7);
        spindexMotorFSM = new spindexMotorFSM(hwMap, telemetry, this.spindexMotor);
        colorSensorsFSM = new ColorSensorFSM(hwMap, telemetry);
        intakeMotorFSM = new IntakeMotorFSM(hwMap, telemetry);
        intakeMotorFSM.state = IntakeMotorFSM.states.INTAKING;
        pidChanges = new PIDChanges(hwMap, telemetry, this.spindexMotor);
        this.telemetry = telemetry;
        this.mode = modes.INTAKNG;
        this.state = states.STOPPING_AT_TARGET;
    }
    // private ElapsedTime modeLockTimer = new ElapsedTime();
    // private final double FLICKER_TIME = 0.3; // 300 milliseconds to prevent code being wonky because of sensor flicking
    public void updateState(double runtime, int r, int g, int b, Gamepad gamepad1) {
        spindexMotor.readPosition();
        spindexMotorFSM.updateState();
        colorSensorsFSM.updateState();
        colorPocket(spindexMotorFSM.currentIndex, r, g, b);

        if (forceEmpty) {
            pocket1 = status.EMPTY;
            pocket2 = status.EMPTY;
            pocket3 = status.EMPTY;
        }

        boolean allFull = (pocket1 != status.EMPTY && pocket2 != status.EMPTY && pocket3 != status.EMPTY);
        boolean allEmpty = (pocket1 == status.EMPTY && pocket2 == status.EMPTY && pocket3 == status.EMPTY);

        // Mode Transitions
        if (mode == modes.INTAKNG && allFull) {
            mode = modes.SHOOTING;
        } else if (mode == modes.SHOOTING && allEmpty) {
            mode = modes.INTAKNG;
            timer.reset();
        }


        switch (mode) {
            case INTAKNG:
                // spindex pocket movement based on timer
                if (timer.seconds() > 2.5) {
                    if (pocket1 == status.EMPTY) {
                        pidChanges.targetAngle = 360;
                        timer.reset();
                    } else if (pocket2 == status.EMPTY) {
                        pidChanges.targetAngle = 120;
                        timer.reset();
                    } else if (pocket3 == status.EMPTY) {
                        pidChanges.targetAngle = 240;
                        timer.reset();
                    }
                }
                //in init  make it so that a pocket mouth is against the intake
                // spindex pocket movement based on color sensor data/detection-Moves when pocket is full
                if(pocket1 == status.EMPTY){
                    pidChanges.targetAngle = 0;
                } else if (pocket2 == status.EMPTY) {
                    pidChanges.targetAngle = 120;
                } else if (pocket3 == status.EMPTY){
                    pidChanges.targetAngle = 240;
                } else{
                    mode = modes.SHOOTING;
                }

                // switch between on and off intake
                intake_OFF_ON(gamepad1);
                //switch between ejecting and intaking
                intake_intake_eject(gamepad1);
                pidChanges.PIDMoveCalc(runtime);
                break;

            case SHOOTING:
                if (intakeEnabled) {
                    intakeMotorFSM.state = IntakeMotorFSM.states.EJECTING;
                } else {
                    intakeMotorFSM.state = IntakeMotorFSM.states.OFF;
                }

                switch (state) {

                    case STOPPING_AT_TARGET:
                        // later
                        break;
                    case STOPPED_AT_TARGET:
                        // later
                        break;
                }

                spindexMotor.set(1.0); // Continuous spin for shooting i guess
                break;
        }
//1
        intakeMotorFSM.updateState();
        updateTelemetry();
    }
    public void updateTelemetry(){
        telemetry.addData("State:", state);
        telemetry.addData("Power:", spindexMotor.get());
        telemetry.addData("Pocket 1 [0]", pocket1);
        telemetry.addData("P1 RGB", rVals[0] + ", " + gVals[0] + ", " + bVals[0]);
        telemetry.addData("Pocket 2 [1]", pocket2);
        telemetry.addData("P2 RGB", rVals[1] + ", " + gVals[1] + ", " + bVals[1]);
        telemetry.addData("Pocket 3 [2]", pocket3);
        telemetry.addData("P3 RGB", rVals[2] + ", " + gVals[2] + ", " + bVals[2]);
        telemetry.addData("Current Index", spindexMotorFSM.currentIndex);
        telemetry.addData("Target Angle", pidChanges.targetAngle);
        telemetry.addData("Current Angle", pidChanges.currentPosition);
    }
    private status color(int pocketIndex) {
        String motif = colorSensorsFSM.getDetectedMotif();
        if (motif.equals("Green")) return status.GREEN;
        else if (motif.equals("Purple")) return status.PURPLE;
        else return status.EMPTY;
    }
    private void intake_OFF_ON(Gamepad gamepad1){
        if (gamepad1.triangle && !lastTriangle) {
            intakeEnabled = !intakeEnabled;
        }
        lastTriangle = gamepad1.triangle;

        if (intakeEnabled) {
            if (mode == modes.INTAKNG) {
                intakeMotorFSM.state = IntakeMotorFSM.states.INTAKING;
            } else {
                intakeMotorFSM.state = IntakeMotorFSM.states.EJECTING;
            }
        } else {
            intakeMotorFSM.state = IntakeMotorFSM.states.OFF;
        }

    }
    private void intake_intake_eject(Gamepad gamepad1) {
        // Debounce the square button
        if (gamepad1.square && !lastSquare) {
            manualEject = !manualEject;
        }
        lastSquare = gamepad1.square;

        // Execute logic based on toggle state
        if (intakeEnabled) {
            if (manualEject) {
                intakeMotorFSM.state = IntakeMotorFSM.states.EJECTING;
            } else {
                // Default Auto Logic
                if (mode == modes.INTAKNG) {
                    intakeMotorFSM.state = IntakeMotorFSM.states.INTAKING;
                } else {
                    intakeMotorFSM.state = IntakeMotorFSM.states.EJECTING;
                }
            }
        } else {
            intakeMotorFSM.state = IntakeMotorFSM.states.OFF;
        }
    }
    private void colorPocket(int pocket, int r, int g, int b) {
        if(pocket == 1) { pocket1 = color(0); rVals[0] = r; gVals[0] = g; bVals[0] = b; }
        else if(pocket == 2) { pocket2 = color(1); rVals[1] = r; gVals[1] = g; bVals[1] = b; }
        else if(pocket == 3) { pocket3 = color(2); rVals[2] = r; gVals[2] = g; bVals[2] = b; }
    }
}