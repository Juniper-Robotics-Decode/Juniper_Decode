package org.firstinspires.ftc.teamcode.shooter.testClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.MainAuto;
import org.firstinspires.ftc.teamcode.core.RobotSettings;
import org.firstinspires.ftc.teamcode.core.Pinpoint;

@TeleOp
public class PinpointDistanceTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        MainAuto.ALLIANCE = "RED";

        RobotSettings robotSettings = new RobotSettings();
        HWMap hwMap = new HWMap(hardwareMap);
        Pinpoint pinpoint = new Pinpoint(hwMap, robotSettings);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Alliance" + MainAuto.ALLIANCE);
        telemetry.addData("start Pos", robotSettings.startPosState.getPose2D());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();

            telemetry.addData("RAW X", pinpoint.getX());
            telemetry.addData("RAW Y", pinpoint.getY());
            telemetry.addData("RAW Heading", pinpoint.getHeading());

            telemetry.addLine("-----------------");

            telemetry.addData("Distance to Goal", pinpoint.getGoalDistance());
            telemetry.addData("Heading Error", pinpoint.getHeadingErrorTrig());

            String turnDir;
            if (pinpoint.getHeadingErrorTrig() > 0) {
                turnDir = "LEFT";
            }
            else {
                turnDir = "RIGHT";
            }
            if (Math.abs(pinpoint.getHeadingErrorTrig()) < 2.0) {
                turnDir = "AIMED";
            }

            telemetry.addData("Suggested Turn", turnDir);

            telemetry.update();
        }
    }
}
