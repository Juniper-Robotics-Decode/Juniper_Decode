package org.firstinspires.ftc.teamcode.Swerve.Drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;

@Config
public class SwerveDrivetrain {

    public enum States { DRIVING, WAITING_TO_LOCK, LOCKED }

    public SwerveModule frontLeftModule, frontRightModule, backRightModule, backLeftModule;
    public SwerveModule[] modules;
    private Logger logger;

    private double[] ws = new double[4];
    private double[] wa = new double[4];
    private double[] lastwa = new double[4];
    private double[] MotorScaling = new double[]{1, 1, -1, 1};

    private double trackwidth = 9.921;
    private double wheelbase = 9.927;
    private double R;

    private States state = States.DRIVING;
    public static double LOCK_DELAY = 0;
    private final ElapsedTime lockTimer = new ElapsedTime();
    private long lastInputTime = 0;

    private double targetheading = 0.0;
    private boolean headingLocked = false;
    private final PIDFController headingController = new PIDFController(0.25, 0, 0, 0);

    public static double P = -0.275, I = 0, D = 0.01, F = 0;

    public SwerveDrivetrain(HWMap hwMap, Logger logger) {
        this.logger = logger;

        frontLeftModule = new SwerveModule(hwMap.FLM, hwMap.FLS, hwMap.FLE, 2.3, false, logger);
        frontRightModule = new SwerveModule(hwMap.FRM, hwMap.FRS, hwMap.FRE, -1.5, false, logger);
        backRightModule = new SwerveModule(hwMap.BRM, hwMap.BRS, hwMap.BRE, 2.6, false, logger);
        backLeftModule = new SwerveModule(hwMap.BLM, hwMap.BLS, hwMap.BLE, 1, false, logger);

        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (SwerveModule m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        R = hypot(trackwidth, wheelbase);
    }

    public void setPose(Pose pose, Double botheading, Double voltage) {
        double driveMagnitude = hypot(pose.x, pose.y);

        long currentTime = System.currentTimeMillis();
        double dt = (lastInputTime == 0) ? 30 : (currentTime - lastInputTime);
        lastInputTime = currentTime;

        boolean hasInput = (driveMagnitude > 0.01 || Math.abs(pose.heading) > 0.01);

        switch (state) {
            case DRIVING:
                drive(pose.x, pose.y, pose.heading, dt, botheading, voltage, driveMagnitude);
                if (!hasInput) {
                    lockTimer.reset();
                    state = States.WAITING_TO_LOCK;
                }
                break;

            case WAITING_TO_LOCK:
                ws = new double[]{0, 0, 0, 0};
                System.arraycopy(lastwa, 0, wa, 0, 4);
                if (hasInput) state = States.DRIVING;
                else if (lockTimer.milliseconds() > LOCK_DELAY) state = States.LOCKED;
                break;

            case LOCKED:
                ws = new double[]{0, 0, 0, 0};
                wa = new double[]{0, 0, 0, 0};
                if (hasInput) state = States.DRIVING;
                break;
        }

        normalizeWheelSpeeds();
        updateModules();
    }

    private void drive(double x, double y, double heading, double dt, double botheading, double voltage, double driveMagnitude) {
        // --- HEADING PID LOGIC ---
        if (Math.abs(heading) > 0.02) {
            headingLocked = false;
        } else if (driveMagnitude > 0.02) {
            if (!headingLocked) {
                targetheading = botheading;
                headingLocked = true;
                headingController.reset();
            }

            double error = normalizeRadians(targetheading - botheading);
            headingController.setPIDF(P, I, D, F);
            heading = headingController.calculate(0, error) * (12.0 / voltage);
        }

        double rotCorr = heading * (dt / 1000.0) / 2.0;
        double cosC = Math.cos(rotCorr);
        double sinC = Math.sin(rotCorr);

        double xCompensated = x * cosC - y * sinC;
        double yCompensated = x * sinC + y * cosC;

        double a = xCompensated - heading * (wheelbase / R),
                b = xCompensated + heading * (wheelbase / R),
                c = yCompensated - heading * (trackwidth / R),
                d = yCompensated + heading * (trackwidth / R);

        ws = new double[]{hypot(a, c), hypot(a, d), hypot(b, d), hypot(b, c)};
        wa = new double[]{atan2(a, c), atan2(a, d), atan2(b, d), atan2(b, c)};
    }

    public void updateModules() {
        for (int i = 0; i < 4; i++) {
            modules[i].update(wa[i], (ws[i] * MotorScaling[i]));
            lastwa[i] = wa[i];
        }
    }

    public void setOffsets(double[] offsets) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setOffset(offsets[i]);
        }
    }

    public void setInverses(boolean[] inverses) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setInverse(inverses[i]);
        }
    }

    public void setMotorScaling(double[] scalers) {
        if (scalers.length == 4) {
            for (int i = 0; i < 4; i++) {
                this.MotorScaling[i] = scalers[i];
            }
        }
    }

    public void setHeadingControllerPIDF(double P, double I, double D, double F) {
        SwerveDrivetrain.P = P;
        SwerveDrivetrain.I = I;
        SwerveDrivetrain.D = D;
        SwerveDrivetrain.F = F;
    }

    public void log() {
        logger.log("Drivetrain State", state.toString(), Logger.LogLevels.PRODUCTION);
        logger.log("Heading Target", targetheading, Logger.LogLevels.DEBUG);
        for (int i = 0; i < modules.length; i++) {
            modules[i].log(i);
        }
    }

    private void normalizeWheelSpeeds() {
        double max = 0;
        for (double speed : ws) max = Math.max(max, speed);
        if (max > 1.0) {
            for (int i = 0; i < 4; i++) ws[i] /= max;
        }
    }
}