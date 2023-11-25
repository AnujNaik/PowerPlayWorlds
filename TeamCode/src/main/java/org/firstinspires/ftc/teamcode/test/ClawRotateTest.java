package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Pooja;


@TeleOp(group = "Test")
public class ClawRotateTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pooja pooja = new Pooja(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.dpad_up) {
                pooja.rotateClawToTransfer();
            }
            if (gamepad1.dpad_down) {
                pooja.rotateClawToIntakeTeleOp();
            }
            if (gamepad1.dpad_left) {
                pooja.rotateClawToIntakeLeftAuto();
            }
            if (gamepad1.dpad_right) {
                pooja.rotateClawToIntakeRightAuto();
            }
        }
    }
}
