package org.firstinspires.ftc.teamcode.PID;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Pooja;

@Disabled
@Config
@TeleOp(group = "Test")
public class kFTuner extends LinearOpMode {

    public static double kF = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pooja pooja = new Pooja(hardwareMap);

        waitForStart();

        while (!isStopRequested()){
            pooja.setCollectorSlidesPower(kF);
        }
    }
}
