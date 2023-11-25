package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.Pooja;

@Disabled
@TeleOp(group = "Test")
public class CollectorEncoderTest extends LinearOpMode {
    private DcMotorEx collectorTop, collectorBottom;

    @Override
    public void runOpMode() throws InterruptedException {
        Pooja pooja = new Pooja(hardwareMap);

        collectorTop = hardwareMap.get(DcMotorEx.class, "collectorTop");
        collectorBottom = hardwareMap.get(DcMotorEx.class, "collectorBottom");

//        collectorTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        collectorTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        collectorTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        collectorBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        collectorBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        collectorBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            double pow = gamepad1.left_stick_y;

            collectorTop.setPower(pow);
            collectorBottom.setPower(pow);

            telemetry.addData("Position", collectorTop.getCurrentPosition());
            telemetry.addData("Power", pow);
            telemetry.update();
        }
    }
}
