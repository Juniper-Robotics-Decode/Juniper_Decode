package org.firstinspires.ftc.teamcode.pedroPathing;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.GamepadManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Configurable
public class TeleopTest {
    SwerveModuleTele lf = new SwerveModuleTele();
    SwerveModuleTele lr = new SwerveModuleTele();
    SwerveModuleTele rf = new SwerveModuleTele();
    SwerveModuleTele rr = new SwerveModuleTele();
    public GoBildaPinpointDriver pinPoint;
    double theta, power, turn, realTheta;
    public double currentHeading = 0, heading = 0;
    PolarVector lfResult, lrResult, rfResult, rrResult;
    double headingError = 0, lastHeadingError = 0, headingCorrection = 0;
    ElapsedTime dt = new ElapsedTime();
    public static double baseKP = -1.5, baseKD = -0.15;

    public void init(HardwareMap hardwareMap) {
        lf.init(hardwareMap, "FL");
        lr.init(hardwareMap, "BL");
        rf.init(hardwareMap, "FR");
        rr.init(hardwareMap, "BR");
        pinPoint = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        lf.setMaxMinVoltage(3.3, 0.0);
        lr.setMaxMinVoltage(3.3, 0.0);
        rf.setMaxMinVoltage(3.3, 0.0);
        rr.setMaxMinVoltage(3.3, 0.0);
        lf.setEncoderOffset(-3.2);
        lr.setEncoderOffset(-1.1);
        rf.setEncoderOffset(1);
        rr.setEncoderOffset(0.9);
        lr.setServoDirection(false);
        rr.setServoDirection(false);
        pinPoint.setOffsets(14.074, -132.5, DistanceUnit.MM);
        pinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinPoint.resetPosAndIMU();
        pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 103.325, 55.325, AngleUnit.DEGREES, 0));
    }

    public void drive(Gamepad gamepad, boolean aim, double target) {
        drive(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, aim, target);
    }

    public void drive(GamepadManager gamepad, boolean aim, double target) {
        drive(-gamepad.getLeftStickY(), gamepad.getLeftStickX(), gamepad.getRightStickX(), aim, target);
    }

    public void drive(double y, double x, double rx, boolean aim, double target) {
        pinPoint.update();
        currentHeading = (pinPoint.getHeading(AngleUnit.RADIANS) + Math.PI) % (Math.PI * 2) - Math.PI;
        theta = Math.atan2(x, y);
        power = Math.hypot(x, y);
//        power = 0.01;
        realTheta = ((Math.PI * 2 + currentHeading) + theta - Math.PI) % (Math.PI * 2) - Math.PI;

        headingError = Math.toRadians(target) - currentHeading;
//        correctionKP = baseKP * (power * 0.9 + 0.7);
//        correctionKD = baseKD * (power * 0.9 + 0.7);
        if (headingError > Math.PI)
            headingError -= 2 * Math.PI;
        else if (heading - currentHeading < -Math.PI)
            headingError += 2 * Math.PI;
        headingCorrection = headingError * baseKP + (headingError - lastHeadingError) / dt.seconds() * baseKD;
        dt.reset();
        lastHeadingError = headingError;

        if (aim) {
            turn = headingCorrection;
        } else {
            turn = rx;
        }

        if (Math.abs(y) + Math.abs(x) + Math.abs(turn) < 0.1) {
            lfResult = new PolarVector(0.0001, -Math.PI / 4.0);
            lrResult = new PolarVector(0.0001, Math.PI / 4.0);
            rfResult = new PolarVector(0.0001, -Math.PI / 4.0 * 3.0);
            rrResult = new PolarVector(0.0001, Math.PI / 4.0 * 3.0);
        } else {
            lfResult = new PolarVector(power, realTheta).add(new PolarVector(turn, Math.PI / 4.0));
            lrResult = new PolarVector(power, realTheta).add(new PolarVector(turn, -Math.PI / 4.0));
            rfResult = new PolarVector(power, realTheta).add(new PolarVector(turn, Math.PI / 4.0 * 3.0));
            rrResult = new PolarVector(power, realTheta).add(new PolarVector(turn, -Math.PI / 4.0 * 3.0));
        }

        lf.runModule(lfResult);
        lr.runModule(lrResult);
        rf.runModule(rfResult);
        rr.runModule(rrResult);
    }

    public double getHeadingError() {
        return headingError;
    }

    public double getHeadingCorrection() {
        return headingCorrection;
    }

    public double getTheta() {
        return realTheta;
    }

    public double getTurn() {
        return turn;
    }

    public void drive(double theta, double power) {
        pinPoint.update();
        currentHeading = (pinPoint.getHeading(AngleUnit.RADIANS) + Math.PI) % (Math.PI * 2) - Math.PI;
        turn = 0;
        realTheta = ((Math.PI * 2 + currentHeading) + theta - Math.PI) % (Math.PI * 2) - Math.PI;

//        headingError = heading - currentHeading;
//        correctionKP = baseKP * (power * 0.9 + 0.7);
//        correctionKD = baseKD * (power * 0.9 + 0.7);
//        if (headingError > Math.PI)
//            headingError -= 2 * Math.PI;
//        else if (heading - currentHeading < -Math.PI)
//            headingError += 2 * Math.PI;
//        headingCorrection = headingError * correctionKP + (headingError - lastHeadingError) / dt.seconds() * correctionKD;
//        dt.reset();
//        lastHeadingError = headingError;

//        if (Math.abs(rx) > 0) {
//            headingReset.reset();
//        }
//        if (headingReset.milliseconds() < 200) {
//            heading = currentHeading;
//            headingError = 0;
//            lastHeadingError = 0;
//            headingCorrection = 0;
//        }

//        lf.runModule(new PolarVector(power, realTheta).add(new PolarVector(turn + headingCorrection, Math.PI / 4.0)));
//        lr.runModule(new PolarVector(power, realTheta).add(new PolarVector(turn + headingCorrection, -Math.PI / 4.0)));
//        rf.runModule(new PolarVector(power, realTheta).add(new PolarVector(turn + headingCorrection, Math.PI / 4.0 * 3.0)));
//        rr.runModule(new PolarVector(power, realTheta).add(new PolarVector(turn + headingCorrection, -Math.PI / 4.0 * 3.0)));

        lfResult = new PolarVector(power, realTheta).add(new PolarVector(turn, Math.PI / 4.0));
        lrResult = new PolarVector(power, realTheta).add(new PolarVector(turn, -Math.PI / 4.0));
        rfResult = new PolarVector(power, realTheta).add(new PolarVector(turn, Math.PI / 4.0 * 3.0));
        rrResult = new PolarVector(power, realTheta).add(new PolarVector(turn, -Math.PI / 4.0 * 3.0));


        lf.runModule(lfResult);
        lr.runModule(lrResult);
        rf.runModule(rfResult);
        rr.runModule(rrResult);
    }

    public String getWheelDirection() {
        return "\nlf: " + Math.toDegrees(lf.getWheelDirection()) + "\nlr: " + Math.toDegrees(lr.getWheelDirection()) + "\nrf: " + Math.toDegrees(rf.getWheelDirection()) + "\nrr: " + Math.toDegrees(rr.getWheelDirection());
    }

    public String getWheelResults() {
        return "\nlf: " + lfResult.toString() + "\nlr: " + lrResult.toString() + "\nrf: " + rfResult.toString() + "\nrr: " + rrResult.toString();
    }

    public double getVoltage(int ind) {
        if (ind == 0) return lf.getVoltage();
        else if (ind == 1) return lr.getVoltage();
        else if (ind == 2) return rf.getVoltage();
        else return rr.getVoltage();
    }

    public double getDirection(int ind) {
        if (ind == 0) return Math.toDegrees(lf.getWheelDirection());
        else if (ind == 1) return Math.toDegrees(lr.getWheelDirection());
        else if (ind == 2) return Math.toDegrees(rf.getWheelDirection());
        else return Math.toDegrees(rr.getWheelDirection());
    }

    public Pose2D getPosition() {
        return pinPoint.getPosition();
    }

    public void setTeleOpDrive(double forward, double strafe, double turn) {
        double currentHeading = 0.0;
        double theta = Math.atan2(strafe, forward);
        double power = Math.hypot(strafe, forward);
        double realTheta = ((Math.PI * 2 + currentHeading) + theta - Math.PI) % (Math.PI * 2) - Math.PI;

        lfResult = new PolarVector(power, realTheta).add(new PolarVector(turn, Math.PI / 4.0));
        lrResult = new PolarVector(power, realTheta).add(new PolarVector(turn, -Math.PI / 4.0));
        rfResult = new PolarVector(power, realTheta).add(new PolarVector(turn, Math.PI / 4.0 * 3.0));
        rrResult = new PolarVector(power, realTheta).add(new PolarVector(turn, -Math.PI / 4.0 * 3.0));

        lf.runModule(lfResult);
        lr.runModule(lrResult);
        rf.runModule(rfResult);
        rr.runModule(rrResult);
    }
}

