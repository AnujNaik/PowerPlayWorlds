package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Pooja;


@TeleOp(group = "Test")
public class ClawConeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pooja pooja = new Pooja(hardwareMap);

        waitForStart();

        while (!isStopRequested()){
            if (gamepad1.dpad_up) {
                pooja.openClaw();
            }
            if (gamepad1.dpad_down) {
                pooja.closeClaw();
            }
        }
    }
}
