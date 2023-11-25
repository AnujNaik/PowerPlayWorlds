package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Pooja;

@Disabled
@Config
@TeleOp(group = "Test")
public class ConeStabilizerTest extends LinearOpMode {
    private Servo coneStabilizer;

    public static double coneStabilizerDisengagedPos = 0.3;
    public static double coneStabilizerEngagedPos = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pooja pooja = new Pooja(hardwareMap);
        coneStabilizer = hardwareMap.get(Servo.class, "coneStabilizer");

        waitForStart();

        while (!isStopRequested()){
            if (gamepad1.right_trigger > 0.1) {
                coneStabilizer.setPosition(coneStabilizerEngagedPos);
            }
            if (gamepad1.left_trigger > 0.1) {
                coneStabilizer.setPosition(coneStabilizerDisengagedPos);
            }
        }
    }
}
