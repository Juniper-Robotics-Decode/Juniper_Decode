package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.core.Pinpoint;

@TeleOp
public class FullShooterTest extends LinearOpMode {

    HWMap hwMap;
    Logger logger;
    RobotSettings robotSettings;
    GamepadEx gamepad;
    LauncherFSM launcherFSM;
    Pinpoint pinpoint;
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            logger = new Logger(telemetry);
            hwMap = new HWMap(hardwareMap);
            robotSettings = RobotSettings.load();
            pinpoint = new Pinpoint(hwMap, robotSettings,false);
            gamepad = new GamepadEx(gamepad1);
            launcherFSM = new LauncherFSM(hwMap,telemetry, pinpoint,robotSettings,logger);
        }catch (Exception e) {
            telemetry.addData("Exception", e.getMessage());
            telemetry.update();
        }
        waitForStart();
        while(opModeIsActive()) {
            pinpoint.update();
            gamepad.readButtons();
            launcherFSM.updateState(gamepad1.b,gamepad1.dpad_up,gamepad1.left_bumper,gamepad2.dpad_up,gamepad2.dpad_down,gamepad2.dpad_left,gamepad2.dpad_right,gamepad2.y,gamepad2.a,gamepad2.b,gamepad2.x, gamepad2.left_bumper,gamepad2.right_bumper);
            log();
        }
    }

    private void log() {
        launcherFSM.log();
        telemetry.update();
    }
}
