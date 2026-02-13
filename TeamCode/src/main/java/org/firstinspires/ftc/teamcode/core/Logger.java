package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;



public class Logger {
    public static enum LogLevels {
        DEBUG,
        PRODUCTION,
        DRIVER_DATA
    }

    public static enum MechanismLog {
        INTAKE_TRANSFER,
        LAUNCHER
    }

    private Telemetry telemetry;
    private LogLevels state;
    private MechanismLog mechanismState;

    public Logger(Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        state = LogLevels.PRODUCTION;
        mechanismState = MechanismLog.INTAKE_TRANSFER;
    }

    public void log(String caption, Object data, LogLevels logLevel) {
        switch (state) {
            case DEBUG:
                if (!(logLevel == LogLevels.DRIVER_DATA)) {
                    telemetry.addData(caption, data);

                }
                break;
            case PRODUCTION:
                if (logLevel == LogLevels.PRODUCTION) {
                    telemetry.addData(caption, data);
                }
                break;
            case DRIVER_DATA:
                if (logLevel == LogLevels.DRIVER_DATA) {
                    telemetry.addData(caption, data);
                }
                break;
        }
    }

    public void updateLoggingLevel(boolean D_Pad_Right) {
        if (D_Pad_Right) {
            if (state == LogLevels.PRODUCTION) {
                state = LogLevels.DEBUG;
                telemetry.addData("CURRENT LOGGER STATE", state);
            } else if (state == LogLevels.DEBUG) {
                state = LogLevels.DRIVER_DATA;
                telemetry.addData("CURRENT LOGGER STATE", state);
            } else if(state == LogLevels.DRIVER_DATA)
                state = LogLevels.PRODUCTION;
            telemetry.addData("CURRENT LOGGER STATE", state);
        }
    }

    public void updateMechanismLevel(boolean Left_Bumper) {
        if (Left_Bumper) {
            if (mechanismState == MechanismLog.LAUNCHER) {
                mechanismState = MechanismLog.INTAKE_TRANSFER;
                telemetry.addLine("MECHANISM STATE: INTAKE_TRANSFER");
            } else if (mechanismState == MechanismLog.INTAKE_TRANSFER) {
                mechanismState = MechanismLog.LAUNCHER;
                telemetry.addLine("MECHANISM STATE: LAUNCHER");
            }
        }

        telemetry.addData("Current Logger State", state);
        telemetry.addData("Current Mechanism state", mechanismState);
    }


    public void print() {
        telemetry.update();
    }

    public boolean DEBUG() {
        return state == LogLevels.DEBUG;
    }

    public boolean PRODUCTION() {
        return state == LogLevels.DEBUG;
    }

    public boolean DRIVER_DATA() {
        return state == LogLevels.DEBUG;
    }

    public boolean INTAKE_TRANSFER() {
        return mechanismState == MechanismLog.INTAKE_TRANSFER;
    }

    public boolean LAUNCHER() {return mechanismState == MechanismLog.LAUNCHER;
    }
}
