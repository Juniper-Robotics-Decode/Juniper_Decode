package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "00 CONFIGURATION", group = "Config")
public class ConfigOpMode extends LinearOpMode {

    RobotSettings settings;

    // Helper for toggle logic (prevents super-fast cycling when holding button)
    boolean lastA = false, lastY = false, lastX = false, lastB = false, lastLeft = false, lastRight = false;

    @Override
    public void runOpMode() {
        // Load existing settings so we don't overwrite with defaults immediately
        settings = RobotSettings.load();

        telemetry.addLine("CONFIGURATION MODE");
        telemetry.addLine("Press STOP to Save & Exit");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 1. ALLIANCE SELECTION (A / Y)
            if (gamepad1.a && !lastA) {
                settings.alliance = RobotSettings.Alliance.BLUE;
            }
            if (gamepad1.y && !lastY) {
                settings.alliance = RobotSettings.Alliance.RED;
            }

            // 2. DISTANCE METHOD (X to cycle)
            if (gamepad1.x && !lastX) {
                // Cycle through enum values
                int nextIndex = (settings.distanceMethod.ordinal() + 1) % RobotSettings.DistanceMethod.values().length;
                settings.distanceMethod = RobotSettings.DistanceMethod.values()[nextIndex];
            }

            // 3. START POSITION (Dpad Left/Right)
            if (gamepad1.dpad_right && !lastRight) {
                int nextIndex = (settings.startPosState.ordinal() + 1) % RobotSettings.StartPos.values().length;
                settings.startPosState = RobotSettings.StartPos.values()[nextIndex];
            }
            if (gamepad1.dpad_left && !lastLeft) {
                int prevIndex = (settings.startPosState.ordinal() - 1);
                if (prevIndex < 0) prevIndex = RobotSettings.StartPos.values().length - 1;
                settings.startPosState = RobotSettings.StartPos.values()[prevIndex];
            }

            // Update button states
            lastA = gamepad1.a;
            lastY = gamepad1.y;
            lastX = gamepad1.x;
            lastRight = gamepad1.dpad_right;
            lastLeft = gamepad1.dpad_left;

            // Display Current Settings
            telemetry.addData("Alliance (A=Blue, Y=Red)", settings.alliance);
            telemetry.addData("Method (X)", settings.distanceMethod);
            telemetry.addData("Start Pos (Dpad)", settings.startPosState);
            telemetry.addLine("\nChanges saved automatically on STOP.");
            telemetry.update();
        }

        // SAVE ON STOP
        settings.save();
        telemetry.addData("Status", "Settings Saved!");
        telemetry.update();
    }
}