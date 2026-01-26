package org.firstinspires.ftc.teamcode.Spindex;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class KickerServo extends LinearOpMode {
    public Servo KickerServo;
    public static boolean launch;
    public static boolean ready;
    public void runOpMode() throws InterruptedException {
        KickerServo.setPosition(0);
        KickerServo = hardwareMap.get(Servo.class, "KickerServo");
        waitForStart();
        while(opModeIsActive()){
         if (launch && ready) {
             //Ready is another boolean which turns true when spindex is spun so that wanted first
             // ball is under shooter and ready to shoot
            KickerServo.setPosition(1);
            wait(500);
            KickerServo.setPosition(0);
         } else {
             KickerServo.setPosition(0);
         }
        }
    }
}
