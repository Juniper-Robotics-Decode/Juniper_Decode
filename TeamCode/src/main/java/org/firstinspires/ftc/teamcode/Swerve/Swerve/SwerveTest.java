package org.firstinspires.ftc.teamcode.Swerve.Swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.headingrate;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.inverses;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.offsets;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.scalars;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.xrate;
import static org.firstinspires.ftc.teamcode.Swerve.Swerve.swerveTuningTele.yrate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Point;
import org.firstinspires.ftc.teamcode.Swerve.Geo.Pose;
import org.firstinspires.ftc.teamcode.core.HWMap;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.JoystickScaling;
import org.firstinspires.ftc.teamcode.Swerve.Limiters.SlewRateLimiter;

@Config
@TeleOp
public class SwerveTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private HWMap hwMap;
    private SwerveDrivetrain swerveDrivetrain;

    public double x, y, heading;
    public double BotHeading;
    public boolean locked;

    GoBildaPinpointDriver odo;
    private double Xoffset, Yoffset;
    private Pose2D pos;

    private SlewRateLimiter XRate, YRate, HeadingRate;
    private JoystickScaling StrafingScaler, TurningScaler;

    private boolean HeadingLocked = false;
    private double TargetHeading;
    private PIDFController hController = new PIDFController(0.5, 0, 0.1, 0);
    public static double P,I,D,F;

    public static double[] MotorScalars = new double[]{1,1,1,1};
    public static double[] Zeros = new double[]{-0.4, 1.1, 3.2, 0.6};


    @Override
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        XRate = new SlewRateLimiter(xrate);
        YRate = new SlewRateLimiter(yrate);
        HeadingRate = new SlewRateLimiter(headingrate);
        StrafingScaler = new JoystickScaling();
        TurningScaler = new JoystickScaling();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        Xoffset = 10.5; Yoffset = 1; //find in CAD
        odo.setOffsets(Xoffset, Yoffset);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();
        Pose2D startingpos = new Pose2D(DistanceUnit.CM, 0.0, 0.0, AngleUnit.RADIANS, 0.0);
        odo.setPosition(startingpos);
        odo.recalibrateIMU();

        hwMap = new HWMap(hardwareMap);
        swerveDrivetrain = new SwerveDrivetrain(hwMap);

        swerveDrivetrain.setOffsets(offsets);
        swerveDrivetrain.setInverses(inverses);
        swerveDrivetrain.setMotorScaling(scalars);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            hController.setPIDF(P, I, D, F);
            swerveDrivetrain.setMotorScaling(MotorScalars);
            swerveDrivetrain.setOffsets(Zeros);

            if (gamepad1.options) {
                odo.resetPosAndIMU();
            }

            odo.update();
            pos = odo.getPosition();
            BotHeading = -pos.getHeading(RADIANS);

            if (gamepad1.right_stick_y > 0.25){
                HeadingLocked = true;
                TargetHeading = Math.PI;
            }
            if (gamepad1.right_stick_y < -0.25) {
                TargetHeading = 0.0;
            }

            heading = gamepad1.right_stick_x;
            if (heading > 0.005) {
                HeadingLocked = false;
            }

            double error = normalizeRadians(normalizeRadians(TargetHeading) - BotHeading);
            double HeadingCorrection = -hController.calculate(0, error);

            Pose drive = new Pose((StrafingScaler.ScaleVector(new Point(gamepad1.left_stick_x, -gamepad1.left_stick_y))), (-TurningScaler.Scale(heading, 0.01, 0.66, 4)));
            if (drive.x == 0 && drive.y == 0 && drive.heading == 0) {
                locked = true;
            }
            else {
                locked = false;
            }

            swerveDrivetrain.setLocked(locked);
            swerveDrivetrain.setPose(drive);
            swerveDrivetrain.updateModules();

            telemetry.addData("Bot Heading", BotHeading);
            telemetry.addData("Tele Module \n",swerveDrivetrain.getTele());
            telemetry.update();
        }

    }

}