package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.Pooja;

@Disabled
@TeleOp(group = "Test")
public class DropperEncoderTest extends LinearOpMode {
    private DcMotorEx dropperFront, dropperBack;

    @Override
    public void runOpMode() throws InterruptedException {
        Pooja pooja = new Pooja(hardwareMap);

        dropperFront = hardwareMap.get(DcMotorEx.class, "dropperFront");
        dropperBack = hardwareMap.get(DcMotorEx.class, "dropperBack");

        dropperFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dropperFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dropperFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dropperBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dropperBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dropperBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()){
            double pow = gamepad1.left_stick_y;

            dropperFront.setPower(pow);
            dropperBack.setPower(pow);

            telemetry.addData("Position", dropperBack.getCurrentPosition());
            telemetry.addData("Power", pow);
            telemetry.update();
        }
    }
}
