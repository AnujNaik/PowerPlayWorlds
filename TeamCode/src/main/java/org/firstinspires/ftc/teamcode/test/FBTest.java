package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Pooja;

@Config
@TeleOp(group = "Test")
public class FBTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pooja pooja = new Pooja(hardwareMap);

        waitForStart();

        while (!isStopRequested()){
            if (gamepad1.dpad_down) {
                pooja.FBToIntake(1);
            }
            if (gamepad1.dpad_up) {
                pooja.FBToTransfer();
            }
            if (gamepad1.dpad_left || gamepad1.dpad_right) {
                pooja.FBToRest();
            }
        }
    }
}
