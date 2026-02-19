/*
package org.firstinspires.ftc.teamcode.Spindex;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.HWMapSpindex;
import org.firstinspires.ftc.teamcode.core.MotorWrapper;

public class PIDChanges {
    private TouchSensorMotorFSM touchSensorMotorFSM;
    private Telemetry telemetry;
    public int x, y, target;
    public MotorWrapper spindexMotor;

    double P = 0.012;
    double D = 0.00013;
    double F = 0.0;
    double velP = 0.002;
    double velI = 0.01;
    double velD = 0.0001;
    double velF = 1.0 / 245.0;
    public double currentPosition;

    double tolerance = 1.5;
    double CPR = 537.7; // gobilda 5203 series
    double velIntegral = 0;
    double velLastError = 0;
    double lastTime = 0;
    double lastIndexTime = 0;
    int pocket = 0;
    public double targetAngle = 0;
    boolean velocityMode = false;
    boolean lastDpadState = false;

    public PIDChanges(HWMapSpindex hwMap, Telemetry telemetry) {
        touchSensorMotorFSM = new TouchSensorMotorFSM(hwMap, telemetry);
        this.telemetry = telemetry;
        this.spindexMotor = touchSensorMotorFSM.spindexMotor;
    }

    public void method(double runtime){
        spindexMotor.resetEncoder();
        lastTime = runtime;
        //  lastIndexTime = runtime;
//            boolean dpadPressed = gamepad1.dpad_down;
//            if (dpadPressed && !lastDpadState) {
//                velocityMode = !velocityMode;
//                if (!velocityMode) {
//                    targetAngle = (wheel.getCurrentPosition() / CPR) * 360.0;
//                }
//                velIntegral = 0;
//                velLastError = 0;
//            }
//            lastDpadState = dpadPressed;
//            if (velocityMode) {
//                PIDVel(40);
//            } else {
        //   telemetry.addData("Mode", velocityMode ? "Velocity" : "Position");
        //   telemetry.addData("Pocket", pocket);
        //  telemetry.addData("Target Angle", targetAngle);
        spindexMotor.set(0);
    }

    public void PIDMoveCalc(double runtime) {
        currentPosition = (spindexMotor.readandGetPosTicks() / CPR) * 360.0;

        double error = targetAngle - currentPosition;
        if (Math.abs(error) <= tolerance && runtime - lastIndexTime > 0.5) {
            pocket++;
            if (pocket >= 3) pocket = 0;

            targetAngle += 120;
            lastIndexTime = runtime;
        }
        PIDPos(targetAngle);
    }

    public void PIDVel(double target, double runtime) {
        double currentTime = runtime;
        double timeChange = currentTime - lastTime;
        if (timeChange <= 0) timeChange = 0.004;
        double currentVelocity = spindexMotor.getVelocity();
        double errorP = target - currentVelocity;
        velIntegral = Range.clip(velIntegral + (errorP * timeChange), -100, 100); // Tighter clip on integral
        double errorD = (errorP - velLastError) / timeChange;
        double correction = (velP * errorP) + (velI * velIntegral) + (velD * errorD) + (velF * target);
        spindexMotor.set(Range.clip(correction, -1.0, 1.0));
        velLastError = errorP;
        lastTime = currentTime;
        telemetry.addData("Vel Target", target);
        telemetry.addData("Vel Actual", currentVelocity);
    }

    public void PIDPos(double target) {
        double currentPos = (spindexMotor.readandGetPosTicks() / CPR) * 360.0;
        double error = target - currentPos;
        double velocity = spindexMotor.getVelocity();
        double correction = (P * error) - (D * velocity);
        double minPower = 0.18;
        if (Math.abs(error) > tolerance) {
            if (correction > 0) correction += minPower;
            else if (correction < 0) correction -= minPower;
        }
        spindexMotor.set(Range.clip(correction, -0.3, 0.3));
        telemetry.addData("Pos Error", error);
        telemetry.addData("Motor Power", correction);
    }
}
*/
