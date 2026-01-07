package org.firstinspires.ftc.teamcode.Swerve.Swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Swerve.Geo.MathUtils;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import org.firstinspires.ftc.teamcode.core.HWMap;

import java.util.Locale;

@Config
public class SwerveDrivetrain {

    public SwerveModule frontLeftModule, frontRightModule, backRightModule, backLeftModule;
    public SwerveModule[] modules;

    private double[] ws = new double[4];
    private double MotorScaling[] = new double[]{1,1,1,1}; //use SwerveCalibration //run with encoder and use ratio of velocities to find
    private double max;

    double[] wa = new double[4];
    double[] lastwa = new double[4];
    private double kgain = 2;
    private double offsets[] = new double[]{2, 2.6, 1.17, 3.4}; //use SwerveCalibration to find
    private boolean inverses[] = new boolean[]{false,false,false,false};

    private double trackwidth = 13.0; //CC distances of modules //find in CAD
    private double wheelbase = 13.0;  // trackwidth is along the width of the robot wheel base is along the length
    private double R;

    double targetheading = 0.0;
    private boolean headingLocked = false;
    private PIDFController headingController = new PIDFController(0.25, 0, 0, 0);
    private double P,I,D,F;

    private boolean locked = false;
    private double lockdelay = 500;
    private boolean waitingtolock;

    private long lastUpdateTime = 0;
    private long lastInputTime = 0;

    public SwerveDrivetrain(HWMap hwMap) {
        frontLeftModule = new SwerveModule(hwMap.FLM, hwMap.FLS, hwMap.FLE, offsets[0], false);
        frontRightModule = new SwerveModule(hwMap.FRM, hwMap.FRS, hwMap.FRE, offsets[1], false);
        backRightModule = new SwerveModule(hwMap.BRM, hwMap.BRS, hwMap.BRE, offsets[2], false);
        backLeftModule = new SwerveModule(hwMap.BLM, hwMap.BLS, hwMap.BLE, offsets[3], false);

        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (SwerveModule m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        R = hypot(trackwidth, wheelbase);
    }

    /// calculates swerve modules motor powers and wheel angles from gamepad inputs and robot current heading as according to 2nd order swerve kinematics
    public void setPose (Pose pose, Double botheading){
        double x = pose.x;
        double y = pose.y;
        double heading = pose.heading;

        // todo add voltage compensation
        /// heading lock logic
        if (Math.abs(pose.heading) > 0.02){
            headingLocked = false;
        }
        else if ((Math.abs(pose.x) > 0.02 || Math.abs(pose.y) > 0.02) && Math.abs(heading) < 0.02){
            if (!headingLocked){ //prevents error from staying at 0 no matter what
                targetheading = botheading;
                headingLocked = true;
                headingController.reset();
            }

            double error = targetheading - botheading;
            headingController.setPIDF(P,I,D,F);
            heading = -headingController.calculate(error,0);
        }

        /// locking logic
        long currentTime = System.currentTimeMillis();
        if (x != 0 || y != 0 || heading != 0) {
            lastInputTime = currentTime;
            locked = false;
        } else {
            if (currentTime - lastInputTime > lockdelay) { //subtract last looptime from lock delay time?
                locked = true;
                waitingtolock = false;
            }
            else {
                waitingtolock = true;
            }
        }

        if (locked) { //locked wheel angles
            ws = new double[]{0,0,0,0};
            wa = new double[]{atan2(-1, -1), atan2(-1, 1), atan2(1, 1), atan2(1, -1)};
        }
        else {
            if (waitingtolock && (x == 0 && y == 0 && heading == 0)){
                ws = new double[]{0,0,0,0};
                wa = lastwa;
            }
            /// kinematics
            else { //2nd order swerve kinematics bastardized(using motor powers as velocities -> need kgain) //proper 2nd order kinematics would need velocities and accel to be set
                double dt = (lastUpdateTime == 0) ? 30 : (currentTime - lastUpdateTime); //finds last loop time
                lastUpdateTime = currentTime;

                double rotationCorrection = heading * (dt / 1000) / 2.0 * kgain;
                double cos = Math.cos(rotationCorrection);
                double sin = Math.sin(rotationCorrection);

                double xc = x * cos - y * sin;
                double yc = x * sin + y * cos;

                /// standard first order kinematics
                double a = xc - heading * (wheelbase / R),
                        b = xc + heading * (wheelbase / R),
                        c = yc - heading * (trackwidth / R),
                        d = yc + heading * (trackwidth / R);

                ws = new double[]{hypot(a, c), hypot(a, d), hypot(b, d), hypot(b, c)};
                wa = new double[]{atan2(a, c), atan2(a, d), atan2(b, d), atan2(b, c)};
            }
        }

        max = MathUtils.max(ws);
    }

    public void updateModules() {
        for (int i = 0; i < 4; i++) {
            SwerveModule m = modules[i];
            if (Math.abs(max) > 1) ws[i] /= max;
            m.update(wa[i], (ws[i]*MotorScaling[i]));
            lastwa = wa;
        }
    }

    public void updateModule(int i) {
        SwerveModule m = modules[i];
        if (Math.abs(max) > 1) ws[i] /= max;
        m.update(wa[i], ws[i]);
    }

    /// calculates motor scalers based on current draw
    // todo test
    // todo improve alpha filter to ignore values that are not probable
        /* insane idea that would be way to much loop time for ftc
        * simulate the current draw of each motor using robots accel, target velocity, actual velocity, voltage, and position on field
        * to predict motor current and compare this against actual current to scale motors
        * also consider previous inputs and robot's physical state as to make sure high current spikes when changing
        * direction suddenly are still considered probable
        * target and actual current can then be used to calculate a residual and
        * detect low preforming modules which can then be normalized to*/ /// might be ideal for ml
        // create a look up table or 3d surface using accel, velocity, and voltage to compare current to a list of predicted currents so the robot can be simulated offline
    //current sensing may be too noisy to use
    public void calculateCurrentBasedScalers() { // todo get real min values for current draw
        double[] currentDraws = new double[4];
        double maxObservedCurrent = 0;
        boolean[] isModuleValid = new boolean[]{true, true, true, true};

        double[] currentMovingAverage = new double[]{0,0,0,0};
        double alpha = 0.1;

        for (int i = 0; i < 4; i++) {
            double instantCurrent = modules[i].getMotorCurrent();
            currentMovingAverage[i] = (alpha * instantCurrent) + (1.0 - alpha) * currentMovingAverage[i];

            if (Math.abs(ws[i]) > 0.5 && currentMovingAverage[i] < 2) {
                isModuleValid[i] = false;
            }

            if (isModuleValid[i] && currentMovingAverage[i] > maxObservedCurrent) {
                maxObservedCurrent = currentMovingAverage[i];
            }
        }

        if (maxObservedCurrent > 2) {
            for (int i = 0; i < 4; i++) {
                if (isModuleValid[i]) {
                    MotorScaling[i] = currentMovingAverage[i] / maxObservedCurrent;
                } else {
                    MotorScaling[i] = 1.0;
                }

                MotorScaling[i] = Math.max(MotorScaling[i], 0.7);
            }
        } else {
            for (int i = 0; i < 4; i++) MotorScaling[i] = 1.0;
        }
    }

    // todo add encoders to swerve motors and update module code to use them
    // todo test encoder noise
    // all dynamic motor scaling methods may require perfect zeros and much lower levels of backlash
    public void calculateVelocityBasedScalers() {
        double[] actualVelocities = new double[4];
        double minVelocityRatio = 1.0;

        for (int i = 0; i < 4; i++) {
            modules[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            actualVelocities[i] = 0;//Math.abs(modules[i].getVelocity());
        }

        double maxObservedVel = 0;
        for (double v : actualVelocities) {
            if (v > maxObservedVel) {maxObservedVel = v;}
        }

        if (maxObservedVel < 50) {
            for (int i = 0; i < 4; i++) MotorScaling[i] = 1.0;
            return;
        }

        double lowestActual = maxObservedVel;
        for (int i = 0; i < 4; i++) {
            if (actualVelocities[i] < lowestActual && actualVelocities[i] > 10) {
                lowestActual = actualVelocities[i];
            }
        }

        for (int i = 0; i < 4; i++) {
            if (actualVelocities[i] > 0) {
                MotorScaling[i] = lowestActual / actualVelocities[i];

                MotorScaling[i] = Math.min(1.0, Math.max(0.7, MotorScaling[i]));
            }
        }
    }

    public void setOffsets(double[] offsets) {
        frontLeftModule.setOffset(offsets[0]);
        frontRightModule.setOffset(offsets[1]);
        backRightModule.setOffset(offsets[2]);
        backLeftModule.setOffset(offsets[3]);
    }

    public double[] getOffsets() { return offsets;}

    public void setInverses(boolean[] inverses) {
        frontLeftModule.setInverse(inverses[0]);
        frontRightModule.setInverse(inverses[1]);
        backRightModule.setInverse(inverses[2]);
        backLeftModule.setInverse(inverses[3]);
    }

    public boolean[] getInverses() { return inverses;}

    public void setKgain(double kgain){ this.kgain = kgain;}

    public double getKgain() { return kgain;}

    public void setMotorScaling(double[] scalars){ //todo (far future) run with encoder and compare to current based motor scaling
        for (int i = 0; i < 4; i++){
            MotorScaling[i] = scalars[i];
        }
    }

    public double[] getMotorScaling() { return MotorScaling;}

    public void setlockdelay(double lockdelay){ this.lockdelay = lockdelay;}

    public double getLockdelay() {return lockdelay;}

    public void setHeadingControllerPIDF(double P, double I, double D, double F){
        this.P = P; this.I = I; this.D = D; this.F = F;
    }

    public double getP(){ return P;} public double getI(){ return I;} public double getD(){ return D;} public double getF(){return F;}

    public void setTargetheading(double targetheading){ this.targetheading = targetheading;}

    public double getTargetheading(){ return targetheading;}

    public void setLocked(boolean locked){ this.locked = locked;}

    public boolean getLocked() { return  locked;}

    public boolean getheadingLocked(){ return headingLocked;}

    public void setHeadingLocked(boolean headingLocked){ this.headingLocked = headingLocked;}

    public String getTele(){
        return String.format(Locale.ENGLISH, "Front Left Module %s \nFront Right Module %s \nBack Right Module %s \nBack Left Module %s \n %s, \n %s, \n %s, \n %s,",frontLeftModule.getTele(), frontRightModule.getTele(), backRightModule.getTele(), backLeftModule.getTele(), wa[0], wa[1], wa[2], wa[3]);
    }
}
