/*
package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.core.Pinpoint;

@TeleOp
public class FullShooterTest extends LinearOpMode {

    HWMap hwMap;
    RobotSettings robotSettings;
    GamepadEx gamepad;
    ShooterFSM shooterFSM;
    Pinpoint pinpoint;
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            hwMap = new HWMap(hardwareMap);
            robotSettings = new RobotSettings();
            pinpoint = new Pinpoint(hwMap, robotSettings);
            gamepad = new GamepadEx(gamepad1);
            shooterFSM = new ShooterFSM(hwMap,telemetry, pinpoint);
        }catch (Exception e) {
            telemetry.addData("Exception", e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while(opModeIsActive()) {
            pinpoint.update();
            gamepad.readButtons();
            shooterFSM.updateState(gamepad.wasJustPressed(GamepadKeys.Button.B));
            log();
        }
    }

    private void log() {
        shooterFSM.log();
        telemetry.update();
    }
}
*/
