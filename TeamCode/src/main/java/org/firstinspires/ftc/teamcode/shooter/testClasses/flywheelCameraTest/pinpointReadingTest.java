/*
package org.firstinspires.ftc.teamcode.shooter.testClasses.flywheelCameraTest;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class pinpointReadingTest extends LinearOpMode {

    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() throws InterruptedException {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        double Xoffset = -132.5, Yoffset = 14.075;
        odo.setOffsets(Xoffset, Yoffset);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections( GoBildaPinpointDriver.EncoderDirection.FORWARD,  GoBildaPinpointDriver.EncoderDirection.FORWARD);

        waitForStart();
        while (opModeIsActive()) {
            odo.update();
            telemetry.addData("ready", odo.getDeviceStatus());
            telemetry.addData("x", odo.getPosX());
            telemetry.addData("Y", odo.getPosY());
            telemetry.addData("Heading", odo.getHeading());
            telemetry.update();
        }
    }
}
*/
