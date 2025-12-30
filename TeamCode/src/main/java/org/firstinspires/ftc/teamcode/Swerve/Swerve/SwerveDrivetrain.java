package org.firstinspires.ftc.teamcode.Swerve.Swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
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
    private double MotorScaling[] = new double[]{1,1,1,1};
    private double max;

    double[] wa = new double[4];
    double[] cwa = new double[4];
    private double offsets[] = new double[]{3.3, 3.5, 1.17, 3.4};
    private boolean inverses[] = new boolean[]{false,false,false,false};

    private double trackwidth = 13.0; //CC distances of modules //find in CAD
    private double wheelbase = 13.0;  // trackwidth is along the width of the robot wheel base is along the length
    private double R;

    private boolean locked = false;

    public SwerveDrivetrain(HWMap hwMap) {
        frontLeftModule = new SwerveModule(hwMap.FLM, hwMap.FLS, hwMap.FLE, offsets[0], false);
        frontRightModule = new SwerveModule(hwMap.FRM, hwMap.FRS, hwMap.FRE, offsets[1], false);
        backRightModule = new SwerveModule(hwMap.BRM, hwMap.BRS, hwMap.BRE, offsets[2], false);
        backLeftModule = new SwerveModule(hwMap.BLM, hwMap.BLS, hwMap.BLE, offsets[3], false);

        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (SwerveModule m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        R = hypot(trackwidth, wheelbase);
    }

    public void setPose (Pose pose){
        double x = pose.x, y = pose.y, heading = pose.heading;

        if (locked) {
            ws = new double[]{0,0,0,0};
            wa = new double[]{atan2(-1, -1), atan2(-1, 1), atan2(1, 1), atan2(1, -1)};
        }
        else {
            double  a = x - heading * (wheelbase / R),
                    b = x + heading * (wheelbase / R),
                    c = y - heading * (trackwidth / R),
                    d = y + heading * (trackwidth / R);

            ws = new double[]{hypot(a,c), hypot(a, d), hypot(b, d), hypot(b, c)};
            wa = new double[]{atan2(a,c), atan2(a,d), atan2(b,d), atan2(b,c)};
        }

        max = MathUtils.max(ws);
    }

    public void updateModules() {
        for (int i = 0; i < 4; i++) {
            SwerveModule m = modules[i];
            if (Math.abs(max) > 1) ws[i] /= max;
            m.update(wa[i], (ws[i]*MotorScaling[i]));
        }
    }

    public void updateModule(int i) {
        SwerveModule m = modules[i];
        if (Math.abs(max) > 1) ws[i] /= max;
        m.update(wa[i], ws[i]);
    }

    public void setOffsets(double[] offsets) {
        frontLeftModule.setOffset(offsets[0]);
        frontRightModule.setOffset(offsets[1]);
        backRightModule.setOffset(offsets[2]);
        backLeftModule.setOffset(offsets[3]);
    }

    public void setInverses(boolean[] inverses) {
        frontLeftModule.setInverse(inverses[0]);
        frontRightModule.setInverse(inverses[1]);
        backRightModule.setInverse(inverses[2]);
        backLeftModule.setInverse(inverses[3]);
    }

    public void setMotorScaling(double[] scalars){
        for (int i = 0; i < 4; i++){
            MotorScaling[i] = scalars[i];
        }
    }

    public void setLocked(Boolean locked)  {
        this.locked = locked;
    }

    public boolean getLocked() {
        return  locked;
    }

    public String getTele(){
        return String.format(Locale.ENGLISH, "Front Left Module %s \nFront Right Module %s \nBack Right Module %s \nBack Left Module %s \n %s, \n %s, \n %s, \n %s,",frontLeftModule.getTele(), frontRightModule.getTele(), backRightModule.getTele(), backLeftModule.getTele(), wa[0], wa[1], wa[2], wa[3]);
    }
}
