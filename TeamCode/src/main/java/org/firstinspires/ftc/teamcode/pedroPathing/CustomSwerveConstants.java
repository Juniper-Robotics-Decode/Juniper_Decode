package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CustomSwerveConstants {
    public double xVelocity = 48;
    public double yVelocity = 48;
    private double[] convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);
    public Vector frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();

    public  double maxPower = 1;

    public  double motorCachingThreshold = 0.01;
    public  boolean useVoltageCompensation = true;
    public  double nominalVoltage = 13.0;
    public  double staticFrictionCoefficient = 0.02;

    public String leftFrontMotorName = "FLM";
    public String leftRearMotorName = "BLM";
    public String rightFrontMotorName = "FRM";
    public String rightRearMotorName = "BRM";
    public String leftFrontServoName = "FLS";
    public String leftRearServoName = "BLS";
    public String rightFrontServoName = "FRS";
    public String rightRearServoName = "BRS";
    public String leftFrontEncoderName = "FLE";
    public String leftRearEncoderName = "BLE";
    public String rightFrontEncoderName = "FRE";
    public String rightRearEncoderName = "BRE";
    public double lfEncoderMaxVoltage = 3.3;
    public double lrEncoderMaxVoltage = 3.3;
    public double rfEncoderMaxVoltage = 3.3;
    public double rrEncoderMaxVoltage = 3.3;
    public double lfEncoderMinVoltage = 0;
    public double lrEncoderMinVoltage = 0.00;
    public double rfEncoderMinVoltage = 0.00;
    public double rrEncoderMinVoltage = 0.00;
    public double lfEncoderOffset = -3.2;
    public double lrEncoderOffset = -1.1;
    public double rfEncoderOffset = 1;
    public double rrEncoderOffset = 0.9;
    public double lfServoBasePower = 0;
    public double lrServoBasePower = 0;
    public double rfServoBasePower = 0;
    public double rrServoBasePower = 0;
    public double lfRotationKP = 0.1;
    public double lrRotationKP = 0.1;
    public double rfRotationKP = 0.1;
    public double rrRotationKP = 0.1;
    public double lfRotationKI = 0.0;
    public double lrRotationKI = 0.0;
    public double rfRotationKI = 0.0;
    public double rrRotationKI = 0.0;
    public double lfRotationKD = 0.0;
    public double lrRotationKD = 0.0;
    public double rfRotationKD = 0.0;
    public double rrRotationKD = 0.0;
    public  DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public  DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    public  DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public  DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    public  DcMotorSimple.Direction leftFrontServoDirection = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction leftRearServoDirection = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction rightFrontServoDirection = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction rightRearServoDirection = DcMotorSimple.Direction.FORWARD;

    public CustomSwerveConstants() {
        defaults();
    }

    public CustomSwerveConstants xVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    public CustomSwerveConstants yVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
        return this;
    }

    public CustomSwerveConstants maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public CustomSwerveConstants leftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
        return this;
    }

    public CustomSwerveConstants leftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
        return this;
    }

    public CustomSwerveConstants rightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
        return this;
    }

    public CustomSwerveConstants rightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
        return this;
    }

    public CustomSwerveConstants leftFrontMotorDirection(DcMotorSimple.Direction leftFrontMotorDirection) {
        this.leftFrontMotorDirection = leftFrontMotorDirection;
        return this;
    }

    public CustomSwerveConstants leftRearMotorDirection(DcMotorSimple.Direction leftRearMotorDirection) {
        this.leftRearMotorDirection = leftRearMotorDirection;
        return this;
    }

    public CustomSwerveConstants rightFrontMotorDirection(DcMotorSimple.Direction rightFrontMotorDirection) {
        this.rightFrontMotorDirection = rightFrontMotorDirection;
        return this;
    }

    public CustomSwerveConstants rightRearMotorDirection(DcMotorSimple.Direction rightRearMotorDirection) {
        this.rightRearMotorDirection = rightRearMotorDirection;
        return this;
    }

    public CustomSwerveConstants motorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
        return this;
    }

    public CustomSwerveConstants useVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
        return this;
    }

    public CustomSwerveConstants nominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public CustomSwerveConstants staticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        return this;
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public void setXVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public void setYVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
    }

    public Vector getFrontLeftVector() {
        return frontLeftVector;
    }

    public void setFrontLeftVector(Vector frontLeftVector) {
        this.frontLeftVector = frontLeftVector;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public String getLeftFrontMotorName() {
        return leftFrontMotorName;
    }

    public void setLeftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
    }

    public String getLeftRearMotorName() {
        return leftRearMotorName;
    }

    public void setLeftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
    }

    public String getRightFrontMotorName() {
        return rightFrontMotorName;
    }

    public void setRightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
    }

    public String getRightRearMotorName() {
        return rightRearMotorName;
    }

    public void setRightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
    }

    public DcMotorSimple.Direction getLeftFrontMotorDirection() {
        return leftFrontMotorDirection;
    }

    public void setLeftFrontMotorDirection(DcMotorSimple.Direction leftFrontMotorDirection) {
        this.leftFrontMotorDirection = leftFrontMotorDirection;
    }

    public DcMotorSimple.Direction getLeftRearMotorDirection() {
        return leftRearMotorDirection;
    }

    public void setLeftRearMotorDirection(DcMotorSimple.Direction leftRearMotorDirection) {
        this.leftRearMotorDirection = leftRearMotorDirection;
    }

    public DcMotorSimple.Direction getRightFrontMotorDirection() {
        return rightFrontMotorDirection;
    }

    public void setRightFrontMotorDirection(DcMotorSimple.Direction rightFrontMotorDirection) {
        this.rightFrontMotorDirection = rightFrontMotorDirection;
    }

    public DcMotorSimple.Direction getRightRearMotorDirection() {
        return rightRearMotorDirection;
    }

    public void setRightRearMotorDirection(DcMotorSimple.Direction rightRearMotorDirection) {
        this.rightRearMotorDirection = rightRearMotorDirection;
    }

    public double getMotorCachingThreshold() {
        return motorCachingThreshold;
    }

    public void setMotorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
    }

    public void setLeftFrontServoDirection(DcMotorSimple.Direction leftFrontServoDirection) {
        this.leftFrontServoDirection = leftFrontServoDirection;
    }

    public void setLeftRearServoDirection(DcMotorSimple.Direction leftRearServoDirection) {
        this.leftRearServoDirection = leftRearServoDirection;
    }

    public void setRightFrontServoDirection(DcMotorSimple.Direction rightFrontServoDirection) {
        this.rightFrontServoDirection = rightFrontServoDirection;
    }

    public void setRightRearServoDirection(DcMotorSimple.Direction rightRearServoDirection) {
        this.rightRearServoDirection = rightRearServoDirection;
    }

    public void defaults() {
        xVelocity = 48;
        yVelocity = 48;
        convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);
        frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();
        maxPower = 1;
        motorCachingThreshold = 0.01;
        useVoltageCompensation = true;
        nominalVoltage = 13.0;
        staticFrictionCoefficient = 0.1;
        leftFrontMotorName = "FLM";
        leftRearMotorName = "BLM";
        rightFrontMotorName = "FRM";
        rightRearMotorName = "BRM";
        leftFrontServoName = "FLS";
        leftRearServoName = "BLS";
        rightFrontServoName = "FRS";
        rightRearServoName = "BRS";
        leftFrontEncoderName = "FLE";
        leftRearEncoderName = "BLE";
        rightFrontEncoderName = "FRE";
        rightRearEncoderName = "BRE";
        lfEncoderMaxVoltage = 3.3;
        lrEncoderMaxVoltage = 3.3;
        rfEncoderMaxVoltage = 3.3;
        rrEncoderMaxVoltage = 3.3;
        lfEncoderMinVoltage = 0.0;
        lrEncoderMinVoltage = 0.0;
        rfEncoderMinVoltage = 0.0;
        rrEncoderMinVoltage = 0.0;
        lfEncoderOffset = -3.2;
        lrEncoderOffset = -1.1;
        rfEncoderOffset = 1;
        rrEncoderOffset = 0.9;
        lfServoBasePower = 0;
        lrServoBasePower = 0;
        rfServoBasePower = 0;
        rrServoBasePower = 0;
        lfRotationKP = 0.1;
        lrRotationKP = 0.1;
        rfRotationKP = 0.1;
        rrRotationKP = 0.1;
        lfRotationKI = 0.0;
        lrRotationKI = 0.0;
        rfRotationKI = 0.0;
        rrRotationKI = 0.0;
        lfRotationKD = 0.0;
        lrRotationKD = 0.0;
        rfRotationKD = 0.0;
        rrRotationKD = 0.0;
        leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        leftFrontServoDirection = DcMotorSimple.Direction.FORWARD;
        leftRearServoDirection = DcMotorSimple.Direction.FORWARD;
        rightFrontServoDirection = DcMotorSimple.Direction.FORWARD;
        rightRearServoDirection = DcMotorSimple.Direction.FORWARD;
    }
}
