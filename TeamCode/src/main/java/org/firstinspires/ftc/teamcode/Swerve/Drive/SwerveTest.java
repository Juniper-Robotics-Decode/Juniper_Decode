package org.firstinspires.ftc.teamcode.Swerve.Drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.headingrate;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.inverses;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.offsets;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.scalars;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.xrate;
import static org.firstinspires.ftc.teamcode.Swerve.Drive.swerveTuningTele.yrate;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Point;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.core.Logger;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.JoystickScaling;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.SlewRateLimiter;

@Config
@TeleOp
public class SwerveTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Logger logger;
    private HWMap hwMap;
    private SwerveDrivetrain swerveDrivetrain;

    public static double x, y, heading;
    public double BotHeading;
    public boolean locked;

    // --- GoBilda Pinpoint ---
    private GoBildaPinpointDriver odo;
    private Pose2D pos;
    private double Xoffset, Yoffset;

    private SlewRateLimiter XRate, YRate, HeadingRate;
    private JoystickScaling StrafingScaler, TurningScaler;

    public static double[] MotorScalars = new double[]{1,1,1,1};
    public static double[] Zeros = new double[]{2.25,-1.15,-0.8,0.9};

    public static double kgain = 2;
    public static double P, I, D, F;

    @Override
    public void runOpMode() throws InterruptedException {
        P = -0.275;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        logger = new Logger(telemetry);

        XRate = new SlewRateLimiter(xrate);
        YRate = new SlewRateLimiter(yrate);
        HeadingRate = new SlewRateLimiter(headingrate);
        StrafingScaler = new JoystickScaling();
        TurningScaler = new JoystickScaling();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        Xoffset = 10.5; // Set your CAD offset
        Yoffset = 1;    // Set your CAD offset
        odo.setOffsets(Xoffset, Yoffset, DistanceUnit.CM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.CM, 0.0, 0.0, AngleUnit.RADIANS, 0.0));
        odo.recalibrateIMU();

        hwMap = new HWMap(hardwareMap);
        swerveDrivetrain = new SwerveDrivetrain(hwMap, logger);

        swerveDrivetrain.setOffsets(offsets);
        swerveDrivetrain.setInverses(inverses);
        swerveDrivetrain.setMotorScaling(scalars);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double voltage = hwMap.getVoltageSensor().getVoltage();

            swerveDrivetrain.setMotorScaling(MotorScalars);
            swerveDrivetrain.setOffsets(Zeros);
            swerveDrivetrain.setHeadingControllerPIDF(P, I, D, F);

            if (gamepad1.options) {
                odo.resetPosAndIMU();
            }

            odo.update();
            pos = odo.getPosition();
            BotHeading = -pos.getHeading(RADIANS);

            Pose drive = new Pose(
                    StrafingScaler.ScaleVector(new Point(gamepad1.left_stick_x, -gamepad1.left_stick_y)),
                    TurningScaler.Scale(-gamepad1.right_stick_x, 0.01, 0.66, 4)
            );

            drive = new Pose(
                    new Point(XRate.calculate(drive.x), YRate.calculate(drive.y)).rotate(BotHeading),
                    HeadingRate.calculate(drive.heading)
            );

            if (abs(drive.x) < 0.02) drive.x = 0;
            if (abs(drive.y) < 0.02) drive.y = 0;
            if (abs(drive.heading) < 0.02) drive.heading = 0;

            locked = (drive.x == 0 && drive.y == 0 && drive.heading == 0);

            swerveDrivetrain.setPose(drive, BotHeading, voltage);

            telemetry.addData("Bot Heading", Math.toDegrees(BotHeading));
            telemetry.addData("X Pos", pos.getX(DistanceUnit.CM));
            telemetry.addData("Y Pos", pos.getY(DistanceUnit.CM));
            telemetry.addData("Status", odo.getDeviceStatus());

            swerveDrivetrain.log();
            telemetry.update();
        }
    }
}