package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Robot Settings")
public class ConfigOpMode extends LinearOpMode {


    RobotSettings settings;

    boolean lastA = false, lastY = false, lastX = false, lastB = false, lastLeft = false, lastRight = false;

    @Override
    public void runOpMode() {
        settings = RobotSettings.load();
        if(settings == null) {
            settings = new RobotSettings();
        }

        telemetry.addLine("Set Robot Settings");
        telemetry.addLine("Press STOP to Save & Exit");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a && !lastA) {
                settings.alliance = RobotSettings.Alliance.BLUE;
            }
            if (gamepad1.y && !lastY) {
                settings.alliance = RobotSettings.Alliance.RED;
            }

            if (gamepad1.x && !lastX) {
                int nextIndex = (settings.distanceMethod.ordinal() + 1) % RobotSettings.DistanceMethod.values().length;
                settings.distanceMethod = RobotSettings.DistanceMethod.values()[nextIndex];
            }

            if (gamepad1.dpad_right && !lastRight) {
                int nextIndex = (settings.startPosState.ordinal() + 1) % RobotSettings.StartPos.values().length;
                settings.startPosState = RobotSettings.StartPos.values()[nextIndex];
            }
            if (gamepad1.dpad_left && !lastLeft) {
                int prevIndex = (settings.startPosState.ordinal() - 1);
                if (prevIndex < 0) prevIndex = RobotSettings.StartPos.values().length - 1;
                settings.startPosState = RobotSettings.StartPos.values()[prevIndex];
            }

            lastA = gamepad1.a;
            lastY = gamepad1.y;
            lastX = gamepad1.x;
            lastRight = gamepad1.dpad_right;
            lastLeft = gamepad1.dpad_left;

            telemetry.addData("Alliance (A=Blue, Y=Red)", settings.alliance);
            telemetry.addData("Method (X)", settings.distanceMethod);
            telemetry.addData("Start Pos (Dpad)", settings.startPosState);
            telemetry.addLine("\nChanges saved automatically on STOP.");
            telemetry.update();
        }

        settings.save();
        telemetry.addData("Status", "Settings Saved!");
        telemetry.update();
    }
}