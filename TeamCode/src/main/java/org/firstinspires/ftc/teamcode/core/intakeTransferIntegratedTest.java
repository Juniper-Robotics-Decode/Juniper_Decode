package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import org.firstinspires.ftc.teamcode.intake.IntakeFSM;
import org.firstinspires.ftc.teamcode.intake.RollerFSM;
import org.firstinspires.ftc.teamcode.intaketransfer.TransferFSM;
import org.slf4j.LoggerFactory;

@Config
@TeleOp
public class intakeTransferIntegratedTest extends LinearOpMode {



    private TestHardwareMap testHardwareMap;
    private GamepadEx gamepad;
    private IntakeFSM intakeFSM;
    private TransferFSM transferFSM;
    private Logger logger;

    @Override

    public void runOpMode() throws InterruptedException {

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        testHardwareMap = new TestHardwareMap(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        logger = new Logger(telemetry);
        transferFSM = new TransferFSM(testHardwareMap, telemetry);
        intakeFSM = new IntakeFSM(testHardwareMap, telemetry);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.update();
            gamepad.readButtons();
            intakeFSM.updateState(gamepad.wasJustPressed(GamepadKeys.Button.Y), (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)));
            transferFSM.updateState(gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER));

        }
    }
}
